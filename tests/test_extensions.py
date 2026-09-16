import numpy as np
import pytest
import yaml
from gymnasium.utils.env_checker import check_env

from lerobot_env_so101.config import SimConfig, load_config
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.tasks.base import discover


def test_external_task_needs_no_core_changes(tmp_path, monkeypatch):
    (tmp_path / "user_task.py").write_text("""
from lerobot_env_so101.tasks.base import Task, SceneSpec, TaskStatus
class WaitTask(Task):
    def scene(self):
        return SceneSpec([])
    def evaluate(self, env):
        return TaskStatus(success=env.step_index >= 3)
""")
    (tmp_path / "wait.yaml").write_text(
        yaml.safe_dump({"id": "user_wait", "class": "user_task:WaitTask", "instruction": "Wait three steps."})
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    assert "user_wait" in discover([str(tmp_path)])
    env = SO101Env("user_wait", SimConfig(images=False, width=32, height=32), [str(tmp_path)])
    try:
        env.reset(seed=0)
        for _ in range(2):
            assert not env.step(env.applied_action)[2]
        assert env.step(env.applied_action)[4]["is_success"]
    finally:
        env.close()


def test_config_group_toggle_and_validation(tmp_path):
    path = tmp_path / "run.yaml"
    path.write_text(
        "sim:\n  randomization:\n    layout: {enabled: false}\n    size: {enabled: true, scale: [0.8, 1.2]}\n"
    )
    cfg = load_config(path)
    assert not cfg.sim.randomization["layout"]["enabled"]
    assert cfg.sim.randomization["size"]["enabled"]
    with pytest.raises(ValueError):
        load_config(overrides={"sim": {"randomization": {"size": {"scale": [1.5, 0.5]}}}})
    with pytest.raises(ValueError):
        load_config(overrides={"sim": {"physics_hz": 601}})


def test_randomization_groups_are_independent():
    a = load_config(overrides={"sim": {"images": False}, "video": False}).sim
    b = load_config(
        overrides={
            "sim": {
                "images": False,
                "randomization": {"appearance": {"enabled": True}, "camera": {"enabled": True}},
            },
            "video": False,
        }
    ).sim
    x, y = SO101Env(cfg=a), SO101Env(cfg=b)
    try:
        x.reset(seed=17)
        y.reset(seed=17)
        assert x.sampled["objects"] == y.sampled["objects"]
        assert y.sampled["groups"]["front"]
    finally:
        x.close()
        y.close()


def test_gym_contract_and_horizon():
    env = SO101Env(cfg=SimConfig(images=False, width=32, height=32, episode_seconds=0.1))
    try:
        check_env(env, skip_render_check=True)
        env.reset(seed=0)
        for _ in range(3):
            obs, reward, done, truncated, info = env.step(env.applied_action)
        assert truncated and not done
        assert env.observation_space.contains(obs)
        assert np.isfinite(reward)
        with pytest.raises(RuntimeError):
            env.step(env.applied_action)
    finally:
        env.close()
