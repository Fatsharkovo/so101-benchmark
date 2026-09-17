import xml.etree.ElementTree as ET
from dataclasses import asdict, replace

import numpy as np
import pytest

from lerobot_env_so101.camera_profile import (
    FrontCameraPose,
    apply_local_profile,
    load_profile,
    pose_rotation,
    profile_path,
    save_profile,
)
from lerobot_env_so101.camera_tuner import CameraTuningSession
from lerobot_env_so101.config import SimConfig, load_config
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.scene import make_xml
from lerobot_env_so101.tasks.base import make_task


@pytest.fixture
def pose():
    return FrontCameraPose(45, 0, 35, 60, 0, 0, 45)


@pytest.fixture
def config():
    return load_config(overrides={"video": False, "sim": {"images": False, "width": 64, "height": 48}})


def test_save_replace_reload_and_failed_write_preserves_old_file(tmp_path, monkeypatch, pose):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    assert load_profile() is None
    save_profile(pose)
    assert load_profile() == pose
    changed = replace(pose, y_cm=-12, pitch_deg=30, yaw_deg=15, roll_deg=-10, fovy=57)
    save_profile(changed)
    assert load_profile() == changed
    content = profile_path().read_bytes()

    def failed_replace(*args):
        raise OSError("disk full")

    monkeypatch.setattr("lerobot_env_so101.camera_profile.os.replace", failed_replace)
    with pytest.raises(OSError, match="disk full"):
        save_profile(pose)
    assert profile_path().read_bytes() == content
    assert not list(profile_path().parent.glob("*.tmp"))


@pytest.mark.parametrize("content", ["[broken", "front: null", "front: {}", "other: 12"])
def test_invalid_profile_is_reported(tmp_path, content):
    path = tmp_path / "front.yaml"
    path.write_text(content)
    with pytest.raises(ValueError, match="Cannot load camera profile"):
        load_profile(path)


@pytest.mark.parametrize(
    "field,value",
    [("fovy", float("nan")), ("x_cm", float("inf")), ("height_cm", 0), ("fovy", 180), ("pitch_deg", 91)],
)
def test_invalid_numbers_rejected(pose, field, value):
    with pytest.raises(ValueError):
        replace(pose, **{field: value})


@pytest.mark.parametrize(
    "pitch,yaw,roll", [(60, 0, 0), (25, 75, -30), (-30, -100, 160), (90, 45, 90), (-90, 120, -60)]
)
def test_pose_round_trip_and_live_edit_does_not_advance_physics(config, pose, pitch, yaw, roll):
    env = SO101Env("stack_blue_on_red", config.sim)
    try:
        env.reset(seed=0)
        camera_pose = replace(pose, pitch_deg=pitch, yaw_deg=yaw, roll_deg=roll, y_cm=7)
        state = env.snapshot()
        model, data = env.model, env.data
        camera_pose.apply(env)
        assert env.model is model and env.data is data
        assert env.snapshot() == state
        read = FrontCameraPose.from_env(env)
        np.testing.assert_allclose(read.position, camera_pose.position)
        np.testing.assert_allclose(read.rotation, camera_pose.rotation, atol=1e-9)
        assert read.fovy == pose.fovy
    finally:
        env.close()


def test_default_optical_axis(pose):
    np.testing.assert_allclose(-pose.rotation[:, 2], [-0.5, 0, -np.sqrt(3) / 2])
    np.testing.assert_allclose(pose_rotation(0, 0, 0)[:, 1], [0, 0, 1])


def test_latest_profile_overrides_yaml_and_survives_reset_without_affecting_wrist(
    tmp_path, monkeypatch, pose
):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    save_profile(pose)
    cfg = SimConfig(images=False, cameras={"front": {"target": [0, 0, 0], "fovy": 80}, "wrist": {"fovy": 67}})
    cfg.randomization["camera"]["enabled"] = True
    assert apply_local_profile(cfg) == profile_path()
    assert "target" not in cfg.cameras["front"]
    env = SO101Env("stack_blue_on_red", cfg)
    try:
        for seed in (0, 1, 2):
            env.reset(seed=seed)
            actual = FrontCameraPose.from_env(env)
            np.testing.assert_allclose(actual.rotation, pose.rotation, atol=1e-9)
            np.testing.assert_allclose(actual.position, pose.position)
            assert env.model.cam("wrist").fovy[0] == 67
            assert "wrist" in env.sampled["groups"] and "front" not in env.sampled["groups"]
            root = ET.fromstring(env.model_xml)
            front = root.find(".//camera[@name='front']")
            assert float(front.get("fovy")) == pose.fovy
            np.testing.assert_allclose(np.fromstring(front.get("pos"), sep=" "), pose.position)
        save_profile(replace(pose, fovy=58))
        env.reset(seed=3)
        assert env.model.cam("front").fovy[0] == 45  # Running session is frozen.
        new_cfg = SimConfig(images=False)
        apply_local_profile(new_cfg)
        assert new_cfg.cameras["front"]["fovy"] == 58
    finally:
        env.close()


def test_python_scene_uses_same_profile_orientation(pose):
    import mujoco

    cfg = SimConfig(images=False)
    changed = replace(pose, yaw_deg=20, roll_deg=12)
    changed.configure(cfg)
    xml, _ = make_xml(make_task("stack_blue_on_red").scene(), cfg, 0)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.cam("front").xmat.reshape(3, 3), changed.rotation, atol=1e-9)


def test_editor_reopen_reconfigure_save_and_restore(tmp_path, config):
    path = tmp_path / "front.yaml"
    session = CameraTuningSession(config, path)
    try:
        original = session.pose
        session.set_pose(replace(original, x_cm=55, y_cm=-5, fovy=58))
        assert session.dirty
        session.save()
        assert not session.dirty
    finally:
        session.close()
    reopened = CameraTuningSession(config, path)
    try:
        assert reopened.pose.fovy == 58
        reopened.set_pose(replace(reopened.pose, fovy=62))
        reopened.save()
        assert load_profile(path).fovy == 62
        reopened.restore_scene()
        assert reopened.pose == original and reopened.dirty
        reopened.restore_saved()
        assert reopened.pose.fovy == 62 and not reopened.dirty
    finally:
        reopened.close()


def test_editor_corrupt_profile_can_be_reconfigured(tmp_path, config):
    path = tmp_path / "front.yaml"
    path.write_text("front: broken")
    session = CameraTuningSession(config, path)
    try:
        assert session.load_error and path.read_text() == "front: broken"
        session.set_pose(replace(session.pose, fovy=60))
        session.save()
        assert load_profile(path).fovy == 60 and session.load_error is None
    finally:
        session.close()


def test_each_control_changes_rendered_image_without_reset(tmp_path):
    cfg = load_config(
        overrides={
            "video": False,
            "sim": {"images": True, "width": 128, "height": 96, "render_backend": "egl"},
        }
    )
    session = CameraTuningSession(cfg, tmp_path / "front.yaml")
    try:
        initial = session.pose
        image = session.render()
        state = session.env.snapshot()
        for key in asdict(initial):
            session.set_pose(replace(initial, **{key: getattr(initial, key) + 8}))
            assert np.any(session.render() != image), key
            assert session.env.snapshot() == state
    finally:
        session.close()
