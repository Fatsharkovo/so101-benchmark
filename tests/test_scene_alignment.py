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


def test_second_servo_housing_is_frictionless_without_changing_joint_or_upper_arm():
    import xml.etree.ElementTree as ET

    import mujoco

    env = SO101Env(cfg=SimConfig(images=False))
    try:
        env.reset(seed=0)
        model = env.model
        for name in ("second_servo_housing", "second_servo_mount"):
            geom = model.geom(name)
            assert geom.bodyid[0] == model.body("shoulder").id
            np.testing.assert_array_equal(geom.friction, [0, 0, 0])
            assert geom.condim[0] == 1 and geom.priority[0] == 2
        assert model.dof_frictionloss[env.dadr[1]] == pytest.approx(0.052)
        upper_geoms = np.where(model.geom_bodyid == model.body("upper_arm").id)[0]
        assert np.all(model.geom_friction[upper_geoms, 0] > 0)
        # Exercise MuJoCo's actual contact mixing against a high-friction object.
        housing = env.data.geom("second_servo_housing")
        position = housing.xpos + housing.xmat.reshape(3, 3)[:, 0] * (
            model.geom("second_servo_housing").size[0] + 0.009
        )
        root = ET.fromstring(env.model_xml)
        body = ET.SubElement(root.find("worldbody"), "body", pos=" ".join(map(str, position)))
        ET.SubElement(body, "freejoint")
        ET.SubElement(
            body,
            "geom",
            name="friction_probe",
            type="sphere",
            size="0.01",
            mass="0.01",
            friction="5 0.1 0.1",
            condim="6",
            priority="1",
        )
        probe_model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        data = mujoco.MjData(probe_model)
        data.qpos[: model.nq] = env.data.qpos
        mujoco.mj_forward(probe_model, data)
        pair = {probe_model.geom("second_servo_housing").id, probe_model.geom("friction_probe").id}
        contacts = [c for c in data.contact if {c.geom1, c.geom2} == pair]
        assert contacts and all(c.dim == 1 for c in contacts)
    finally:
        env.close()
