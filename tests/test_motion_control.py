import numpy as np
import pytest
from scipy.signal import butter, sosfiltfilt

from lerobot_env_so101.config import SimConfig, load_config
from lerobot_env_so101.env import SO101Env


def test_interpolation_reduces_lifting_ripple_without_large_tracking_error(monkeypatch):
    results = []
    for interpolate in (False, True):
        env = SO101Env("stack_blue_on_red", SimConfig(images=False, interpolate_actions=interpolate))
        try:
            env.reset(seed=0)
            start = np.array([0, -50, 55, 35, 0, 30.0])
            env.data.qpos[env.qadr] = env.to_sim(start)
            env.data.qvel[:] = 0
            env.data.ctrl[env.actuator_ids] = env.to_sim(start)
            for _ in range(600):
                env.mj.mj_step(env.model, env.data)
            positions, targets = [], []
            real_step = env.mj.mj_step

            def sample(model, data):
                real_step(model, data)
                positions.append(data.qpos[env.qadr[:5]].copy())
                for contact in data.contact:
                    bodies = model.geom_bodyid[[contact.geom1, contact.geom2]]
                    assert not any(body in env.robot_bodies for body in bodies)

            with monkeypatch.context() as patch:
                patch.setattr(env.mj, "mj_step", sample)
                for index in range(180):
                    time = index / 30
                    blend = (1 - np.cos(np.pi * (time - 1) / 2)) / 2 if 1 <= time < 5 else 0
                    action = start + np.array([0, -25, -15, 15, 0, 0]) * blend
                    env.step(action, trial=True)
                    targets.extend([env.to_sim(action)[:5]] * 20)
            actual = np.asarray(positions)[600:3000]
            requested = np.asarray(targets)[600:3000]
            ripple = actual - sosfiltfilt(butter(4, 8, fs=600, output="sos"), actual, axis=0)
            results.append(np.sqrt(np.mean(ripple[:, 1:4] ** 2)))
            assert np.rad2deg(np.sqrt(np.mean((actual - requested) ** 2, axis=0))).max() < 0.5
            assert np.rad2deg(abs(np.asarray(positions)[-1] - env.to_sim(start)[:5])).max() < 0.2
        finally:
            env.close()
    assert results[1] < results[0] * 0.1


def test_interpolation_reset_and_action_contract():
    env = SO101Env(cfg=SimConfig(images=False, interpolate_actions=True))
    try:
        env.reset(seed=0)
        action = env.applied_action.copy()
        action[1] -= 3
        action[3] -= 2
        env.step(action, trial=True)
        first_qpos = env.data.qpos.copy()
        first_qvel = env.data.qvel.copy()
        np.testing.assert_allclose(env.data.ctrl[env.actuator_ids], env.to_sim(action))
        np.testing.assert_allclose(env.applied_action, action)
        env.step(action + [0, -5, 0, 2, 0, 0], trial=True)
        env.restore_initial()
        env.step(action, trial=True)
        np.testing.assert_allclose(env.data.qpos, first_qpos, atol=1e-12)
        np.testing.assert_allclose(env.data.qvel, first_qvel, atol=1e-12)
        assert env.data.time == pytest.approx(1 / env.cfg.control_hz)
        assert env.step_index == 0
    finally:
        env.close()


def test_motion_settings_are_explicit_and_validated():
    assert not SimConfig().interpolate_actions
    assert load_config("configs/teleop.yaml").sim.interpolate_actions
    with pytest.raises(ValueError, match="boolean"):
        SimConfig(interpolate_actions="false")
