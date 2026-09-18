"""Physical plate contact, moving-target scoring and bounded teleoperation layouts."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from lerobot_env_so101.config import SimConfig
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.scene import make_xml


@pytest.fixture
def env():
    instance = SO101Env(
        "place_blue_in_plate", SimConfig(images=False, randomization={"layout": {"enabled": False}})
    )
    instance.reset(seed=0)
    yield instance
    instance.close()


def test_plate_contact_push_and_release(env):
    """A contacting finger proxy must translate the plate, which then comes to rest."""
    assert env.model.body("plate").mass[0] == pytest.approx(0.05)
    assert env.scene_spec.plate_rim_height == pytest.approx(0.01)
    initial = env.data.body("plate").xpos.copy()
    root = ET.fromstring(env.model_xml)
    body = ET.SubElement(root.find("worldbody"), "body", name="pusher", mocap="true")
    ET.SubElement(body, "geom", type="box", size="0.005 0.015 0.003", name="pusher_geom")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    data.qpos[:] = env.data.qpos
    data.ctrl[:] = env.data.ctrl
    contact_seen = False
    for i in range(600):
        data.mocap_pos[0] = initial + [-0.061 + 0.0001 * i, 0, 0.007]
        mujoco.mj_step(model, data)
        contact_seen |= any(model.geom("pusher_geom").id in (c.geom1, c.geom2) for c in data.contact)
    assert contact_seen
    assert data.body("plate").xpos[0] - initial[0] > 0.025
    data.mocap_pos[0] = [1, 1, 1]
    for _ in range(1200):
        mujoco.mj_step(model, data)
    velocity = np.empty(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, model.body("plate").id, velocity, 0)
    assert np.linalg.norm(velocity[:3]) < 0.1
    assert np.linalg.norm(velocity[3:]) < 0.01


def test_moved_rotated_plate_scoring_and_restore(env):
    before = env.data.qpos.copy()
    joint = env.model.joint("plate_joint")
    address = joint.qposadr[0]
    env.data.qpos[address : address + 3] += [-0.02, 0.04, 0]
    yaw = 0.4
    env.data.qpos[address + 3 : address + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    mujoco.mj_forward(env.model, env.data)
    target = env.model.joint("blue_joint").qposadr[0]
    env.data.qpos[target : target + 3] = env.data.body("plate").xpos + [0, 0, 0.017]
    env.data.qpos[target + 3 : target + 7] = env.data.qpos[address + 3 : address + 7]
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    assert env.in_plate("blue")
    env.task_impl.grasped = env.task_impl.lifted = True
    # A rotating plate must not be accepted as a stable placement.
    env.data.qvel[joint.dofadr[0] + 5] = 1
    mujoco.mj_forward(env.model, env.data)
    assert not env.task_impl.evaluate(env).success
    assert env.task_impl.stable_steps == 0
    env.data.qvel[:] = 0
    for _ in range(120):
        _, _, done, _, info = env.step(env.applied_action)
        if done:
            break
    assert info["is_success"]
    env.restore_initial()
    np.testing.assert_allclose(env.data.qpos, before, atol=1e-12)


def test_python_plate_matches_xml_physics(env):
    xml, _ = make_xml(env.task_impl.scene(), env.cfg, 0)
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.joint("plate_joint").type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert model.body("plate").mass[0] == pytest.approx(env.model.body("plate").mass[0])
    np.testing.assert_allclose(model.geom("plate_rim_0").size, env.model.geom("plate_rim_0").size)


def test_relaxed_edge_tolerance_still_rejects_rim_and_suspended_cube(env):
    address = env.model.joint("blue_joint").qposadr[0]
    plate = env.data.body("plate")
    rotation = plate.xmat.reshape(3, 3).copy()
    center = plate.xpos.copy()
    env.data.qpos[address + 3 : address + 7] = env.data.qpos[
        env.model.joint("plate_joint").qposadr[0] + 3 : env.model.joint("plate_joint").qposadr[0] + 7
    ]
    tolerance = env.task_impl.params["plate_edge_tolerance"]
    # The outer cube face crosses the inner edge by 1 mm, within the 2 mm allowance.
    env.data.qpos[address : address + 3] = center + rotation @ np.array([0.0365, 0, 0.0145])
    mujoco.mj_forward(env.model, env.data)
    assert not env.in_plate("blue")
    assert env.in_plate("blue", edge_tolerance=tolerance)
    for local in ([0.05, 0, 0.0245], [0, 0, 0.06]):
        env.data.qpos[address : address + 3] = center + rotation @ np.array(local)
        mujoco.mj_forward(env.model, env.data)
        assert not env.in_plate("blue", edge_tolerance=tolerance)


def test_relaxed_hold_accepts_small_motion_but_not_a_fast_plate(env):
    address = env.model.joint("blue_joint").qposadr[0]
    env.data.qpos[address : address + 3] = env.data.body("plate").xpos + [0, 0, 0.014]
    mujoco.mj_forward(env.model, env.data)
    for _ in range(60):
        env.step(env.applied_action)
    env.task_impl.grasped = env.task_impl.lifted = True
    # Evaluate the task clock with a small residual motion below the new thresholds.
    for name in ("blue", "plate"):
        adr = env.model.joint(f"{name}_joint").dofadr[0]
        env.data.qvel[adr : adr + 6] = [0.015, 0, 0, 0, 0, 0.2]
    mujoco.mj_forward(env.model, env.data)
    for _ in range(14):
        assert not env.task_impl.evaluate(env).success
    assert env.task_impl.evaluate(env).success
    adr = env.model.joint("plate_joint").dofadr[0]
    env.data.qvel[adr] = 0.05
    mujoco.mj_forward(env.model, env.data)
    assert not env.task_impl.evaluate(env).success
    assert env.task_impl.stable_steps == 0
