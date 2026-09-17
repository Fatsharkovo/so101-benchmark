import numpy as np
import pytest

from lerobot_env_so101.config import SimConfig
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.tasks.base import discover


def test_wrist_zero_rotates_the_whole_gripper_forward():
    env = SO101Env(cfg=SimConfig(images=False))
    try:
        env.reset(seed=0)
        action = np.array([0, -70, 70, 60, 0, 70], dtype=float)
        q = env.to_sim(action)
        assert q[4] == pytest.approx(-np.pi / 2)
        np.testing.assert_allclose(env.from_sim(q), action, atol=1e-5)
        env.data.qpos[env.qadr] = q
        env.mj.mj_forward(env.model, env.data)
        offset = env.data.cam("wrist").xpos - env.data.body("gripper").xpos
        assert offset[0] > 0.05
        assert abs(offset[1]) < 0.005
        action[4] = 30
        assert env.to_sim(action)[4] - q[4] == pytest.approx(np.pi / 6)
    finally:
        env.close()


@pytest.mark.parametrize("task", list(discover()))
def test_front_camera_pose_and_ten_gram_cubes(task):
    env = SO101Env(task, SimConfig(images=False))
    try:
        env.reset(seed=0)
        np.testing.assert_allclose(env.data.cam("front").xpos, [0.45, 0, 0.35])
        direction = -env.data.cam("front").xmat.reshape(3, 3)[:, 2]
        np.testing.assert_allclose(direction, np.array([-0.5, 0, -np.sqrt(3) / 2]), atol=1e-8)
        for name in env.object_sizes:
            assert env.model.body(name).mass[0] == pytest.approx(0.010)
    finally:
        env.close()


def test_rounded_square_plate_accepts_sides_but_rejects_cut_corners():
    env = SO101Env(cfg=SimConfig(images=False))
    try:
        env.reset(seed=0)
        spec = env.scene_spec
        assert spec.plate_radius * 2 == pytest.approx(0.10)
        assert spec.plate_distance([0.04, 0.02]) < -0.006
        assert spec.plate_distance([0.048, 0.048]) > 0
        name = "red"
        address = env.model.joint(f"{name}_joint").qposadr[0]
        center = np.array([*env.sampled["plate_xy"], 0.019])
        env.data.qpos[address + 3 : address + 7] = [1, 0, 0, 0]
        env.data.qpos[address : address + 3] = center + [0.027, 0.008, 0]
        env.mj.mj_forward(env.model, env.data)
        assert env.in_plate(name)
        env.data.qpos[address : address + 3] = center + [0.033, 0.033, 0]
        env.mj.mj_forward(env.model, env.data)
        assert not env.in_plate(name)
    finally:
        env.close()
