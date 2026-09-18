from collections import deque

import numpy as np
import pytest

from lerobot_env_so101.config import load_config
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.tasks.base import TaskStatus
from lerobot_env_so101.teleop import TeleopSession


class Writer:
    def __init__(self):
        self.events = deque([{"event": "ready"}])
        self.commands = []

    def poll(self):
        events = list(self.events)
        self.events.clear()
        return events

    def submit(self, command, payload=None):
        self.commands.append((command, payload))


@pytest.fixture
def session():
    config = load_config(overrides={"video": False, "sim": {"images": False, "episode_seconds": 0.1}})
    env = SO101Env("stack_blue_on_red", config.sim)
    env.reset(seed=4)
    result = TeleopSession(env, Writer(), config, 4)
    result.poll()
    yield result
    env.close()


def test_trial_motion_and_space_restore_full_initial_state(session):
    env = session.env
    qpos, qvel, ctrl = env.data.qpos.copy(), env.data.qvel.copy(), env.data.ctrl.copy()
    model, data = env.model, env.data
    for _ in range(8):
        session.step(np.array([30, -60, 70, 60, 20, 50]))
    assert not np.allclose(env.data.qpos, qpos)
    assert env.step_index == 0
    assert session.writer.commands == []
    session.start()
    assert env.model is model and env.data is data
    np.testing.assert_array_equal(env.data.qpos, qpos)
    np.testing.assert_array_equal(env.data.qvel, qvel)
    np.testing.assert_array_equal(env.data.ctrl, ctrl)
    assert env.data.time == 0
    assert env.task_impl.stable_steps == 0
    session.step(env.applied_action)
    assert env.step_index == 1
    session.start()
    assert env.step_index == 1  # A second Space during recording must not reset it.
    frame = session.writer.commands[0][1]
    np.testing.assert_allclose(frame["observation.state"], env.from_sim(qpos[env.qadr]))
    np.testing.assert_array_equal(frame["action"], env.applied_action)


def test_count_increases_only_after_save_ack_and_survives_resets(session):
    session.start()
    session.env.task_impl.evaluate = lambda env: TaskStatus(success=True)
    session.step(session.env.applied_action)
    assert session.state == "SAVING"
    assert session.saved_episodes == 0
    assert [c[0] for c in session.writer.commands] == ["frame", "save"]
    session.writer.events.append({"event": "saved", "count": 1})
    session.poll()
    assert session.state == "WAITING" and session.saved_episodes == 1
    assert session.seed == 5
    session.reset()
    assert session.saved_episodes == 1
    assert "Saved episodes: 1 | Space: start recording | R: reset" in session.label()
    session.start()
    session.reset()
    session.writer.events.append({"event": "discarded"})
    session.poll()
    assert session.saved_episodes == 1 and session.seed == 7


@pytest.mark.parametrize("ending", ["timeout", "failure", "reset"])
def test_unsuccessful_recordings_discard_and_advance_layout_after_ack(session, ending):
    before = session.env.sampled
    session.start()
    if ending == "failure":
        session.env.task_impl.evaluate = lambda env: TaskStatus(failure="object_fell")
    for _ in range(3 if ending == "timeout" else 1):
        session.step(session.env.applied_action)
    if ending == "reset":
        session.reset()
    assert session.state == "DISCARDING"
    assert session.writer.commands[-1][0] == "discard"
    assert not any(c[0] == "save" for c in session.writer.commands)
    assert session.seed == 4 and session.env.sampled == before
    session.reset()  # Repeated R while discarding must not discard or reset twice.
    assert len([c for c in session.writer.commands if c[0] == "discard"]) == 1
    session.writer.events.append({"event": "discarded"})
    session.poll()
    assert session.seed == 5 and session.saved_episodes == 0 and session.env.step_index == 0
    assert session.env.sampled["objects"] != before["objects"]
    assert session.state == "WAITING"
    assert session.resets == 1
    session.writer.events.append({"event": "discarded"})
    session.poll()
    assert session.seed == 5 and session.resets == 1


@pytest.mark.parametrize("task", ["place_blue_in_plate", "stack_green_on_yellow"])
def test_each_waiting_reset_resamples_but_space_keeps_new_layout(task):
    config = load_config("configs/teleop.yaml", overrides={"video": False, "sim": {"images": False}})
    env = SO101Env(task, config.sim)
    try:
        env.reset(seed=0)
        writer = Writer()
        session = TeleopSession(env, writer, config, 0)
        session.poll()
        before = env.sampled
        for seed in (1, 2, 3):
            session.reset()
            assert session.seed == seed and env.sampled["seed"] == seed
            for name in env.object_sizes:
                assert env.sampled["objects"][name]["position"] != before["objects"][name]["position"]
            if "plate_xy" in before:
                assert env.sampled["plate_xy"] != before["plate_xy"]
            assert session.state == "WAITING" and env.step_index == 0
            assert session.saved_episodes == 0 and not writer.commands
            before = env.sampled
        initial_qpos = env.data.qpos.copy()
        scene_xml = env.model_xml
        model = env.model
        session.start()
        assert session.seed == 3 and env.sampled == before
        assert env.model is model and env.model_xml == scene_xml
        np.testing.assert_array_equal(env.data.qpos, initial_qpos)
        env.task_impl.evaluate = lambda env: TaskStatus(success=True)
        session.step(env.applied_action)
        payload = writer.commands[-1][1]
        assert payload["seed"] == 3 and payload["sampled"] == before
        assert payload["scene_xml"] == scene_xml
        np.testing.assert_array_equal(payload["initial_state"]["qpos"], initial_qpos)
    finally:
        env.close()


def test_explicit_fixed_layout_config_remains_fixed():
    config = load_config("configs/fixed.yaml", overrides={"video": False, "sim": {"images": False}})
    env = SO101Env("stack_blue_on_red", config.sim)
    try:
        env.reset(seed=0)
        before = env.sampled["objects"]
        session = TeleopSession(env, Writer(), config, 0)
        session.poll()
        session.reset()
        assert session.seed == 1 and env.sampled["objects"] == before
    finally:
        env.close()


def test_writer_error_does_not_count_or_continue(session):
    session.start()
    session.writer.events.append({"event": "error", "message": "disk full"})
    session.poll()
    session.step(session.env.applied_action)
    assert session.saved_episodes == 0 and session.env.step_index == 0
    assert session.state == "ERROR" and "disk full" in session.label()
