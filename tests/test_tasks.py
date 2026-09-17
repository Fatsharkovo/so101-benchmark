import numpy as np
import pytest

from lerobot_env_so101.config import SimConfig
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.tasks.base import discover


def relocate(env, name, position):
    joint = env.model.joint(f"{name}_joint")
    address = joint.qposadr[0]
    env.data.qpos[address : address + 3] = position
    env.data.qpos[address + 3 : address + 7] = [1, 0, 0, 0]
    env.data.qvel[joint.dofadr[0] : joint.dofadr[0] + 6] = 0
    env.mj.mj_forward(env.model, env.data)


@pytest.fixture
def env():
    env = SO101Env(cfg=SimConfig(images=False, width=32, height=32))
    env.reset(seed=2)
    yield env
    env.close()


def test_reset_reproduces_layout_across_color_tasks(env):
    initial = env.snapshot()
    for _ in range(5):
        env.step(env.applied_action)
    env.reset(seed=2)
    np.testing.assert_allclose(env.snapshot()["qpos"], initial["qpos"], atol=1e-10)
    other = SO101Env("place_green_in_plate", SimConfig(images=False, width=32, height=32))
    try:
        other.reset(seed=2)
        assert other.sampled == env.sampled
        assert other.task_description != env.task_description
    finally:
        other.close()


def test_units_clipping_and_nan(env):
    action = np.array([20, -30, 40, 50, -60, 35])
    np.testing.assert_allclose(env.from_sim(env.to_sim(action)), action, atol=1e-5)
    q = env.to_sim(np.array([999] * 6))
    assert np.all(q <= env.limits[:, 1])
    with pytest.raises(ValueError, match="finite"):
        env.step(np.full(6, np.nan))


def settle(env, steps=50):
    result = None
    for _ in range(steps):
        result = env.step(env.applied_action)
        if result[2] or result[3]:
            break
    return result


@pytest.mark.parametrize("color", ["red", "blue", "green", "yellow", "orange"])
def test_each_color_uses_own_target_and_allows_correction(color):
    env = SO101Env(f"place_{color}_in_plate", SimConfig(images=False, width=32, height=32))
    try:
        env.reset(seed=0)
        wrong = "blue" if color != "blue" else "red"
        original = env.object_position(wrong)
        center = np.array([*env.sampled["plate_xy"], 0.02])
        relocate(env, wrong, center)
        assert not settle(env)[4]["is_success"]
        relocate(env, wrong, center + [0.026, 0, 0])
        relocate(env, color, center + [-0.012, 0, 0])
        env.task_impl.grasped = env.task_impl.lifted = True
        assert not settle(env)[4]["is_success"]
        relocate(env, wrong, original)
        result = settle(env, 90)
        assert result[4]["is_success"]
    finally:
        env.close()


def test_placement_without_lift_is_not_success(env):
    relocate(env, "red", [*env.sampled["plate_xy"], 0.02])
    assert not settle(env, 90)[4]["is_success"]


def test_stability_timer_resets_and_rim_is_not_inside(env):
    env.task_impl.grasped = env.task_impl.lifted = True
    relocate(env, "red", [*env.sampled["plate_xy"], 0.02])
    settle(env, 20)
    assert env.task_impl.stable_steps < 30
    xy = env.sampled["plate_xy"]
    relocate(env, "red", [xy[0] + env.scene_spec.plate_radius, xy[1], 0.028])
    env.step(env.applied_action)
    assert env.task_impl.stable_steps == 0
    assert not env.in_plate("red")


def test_stack_requires_support_and_stability():
    env = SO101Env("stack_blue_on_red", SimConfig(images=False, width=32, height=32))
    try:
        env.reset(seed=0)
        env.task_impl.grasped = env.task_impl.lifted = True
        base = env.object_position("red")
        relocate(env, "blue", base + [0, 0, 0.035])
        assert not env.task_impl.evaluate(env).success
        assert settle(env, 120)[4]["is_success"]
    finally:
        env.close()


@pytest.mark.parametrize("task", list(discover()))
@pytest.mark.parametrize("interpolate", [False, True])
def test_scripted_contact_baseline_fixed_scene(task, interpolate):
    env = SO101Env(
        task,
        SimConfig(
            interpolate_actions=interpolate,
            images=False,
            width=32,
            height=32,
            episode_seconds=20,
            randomization={"layout": {"enabled": False}},
        ),
    )
    try:
        env.reset(seed=0)
        oracle = env.task_impl.make_oracle(env)
        for _ in range(env._max_episode_steps):
            _, _, terminated, truncated, info = env.step(oracle.action())
            if terminated or truncated:
                break
        assert info["is_success"], info
    finally:
        env.close()
