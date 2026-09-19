import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

import task_scenes
from lerobot_env_so101.config import SimConfig
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.tasks.base import discover
from lerobot_env_so101.xml_scene import read_xml, scene_path


@pytest.mark.parametrize("task", list(discover()))
def test_scene_source_compiles_directly(task):
    path = scene_path(discover()[task])
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.camera("front").id >= 0


def test_xml_edits_and_explicit_config_overrides(tmp_path):
    root = read_xml(Path(task_scenes.__file__).parent / "place_red_in_plate.xml")
    camera = root.find(".//camera[@name='front']")
    camera.set("pos", "0.28 0.01 0.37")
    root.find(".//geom[@name='red_geom']").set("mass", "0.017")
    root.find(".//body[@name='red']").set("pos", "0.13 -0.10 0.0135")
    mesh = root.find("asset/mesh[@name='plate_base_mesh']")
    vertices = np.fromstring(mesh.get("vertex"), sep=" ").reshape(-1, 3)
    vertices[:, :2] *= 0.9
    mesh.set("vertex", " ".join(map(str, vertices.ravel())))
    path = tmp_path / "edited.xml"
    ET.ElementTree(root).write(path)
    cfg = SimConfig(images=False, randomization={"layout": {"enabled": False}})
    env = SO101Env(cfg=cfg)
    env.task_impl.definition["scene"] = str(path)
    try:
        env.reset(seed=2)
        np.testing.assert_allclose(env.data.cam("front").xpos, [0.28, 0.01, 0.37])
        assert env.model.body("red").mass[0] == pytest.approx(0.017)
        assert env.scene_spec.plate_radius == pytest.approx(0.045)
        assert env.sampled["objects"]["red"]["position"][0] == pytest.approx(0.13)
        cfg.cameras = {"front": {"position": [0.30, 0, 0.35]}}
        env.task_impl.params["cube_mass"] = 0.012
        env.reset(seed=2)
        assert env.model.body("red").mass[0] == pytest.approx(0.012)
        np.testing.assert_allclose(env.data.cam("front").xpos, [0.30, 0, 0.35])
    finally:
        env.close()


def test_replay_uses_captured_scene(tmp_path):
    cfg = SimConfig(images=False)
    env = SO101Env(cfg=cfg)
    try:
        env.reset(seed=2)
        root = ET.fromstring(env.model_xml)
        root.find(".//camera[@name='front']").set("pos", "0.26 0 0.34")
        path = tmp_path / "scene.xml"
        ET.ElementTree(root).write(path)
        env.task_impl.definition["scene"] = str(tmp_path / "no_longer_available.xml")
        env.reset(seed=99, options={"scene_xml": path})
        np.testing.assert_allclose(env.data.cam("front").xpos, [0.26, 0, 0.34])
    finally:
        env.close()


def test_xml_plate_height_and_rotation_are_preserved(tmp_path):
    root = read_xml(Path(task_scenes.__file__).parent / "place_red_in_plate.xml")
    plate = root.find("worldbody/body[@name='plate']")
    plate.remove(plate.find("freejoint"))
    plate.set("pos", "0.14 0.13 0.01")
    plate.set("euler", "0 0 0.3")
    path = tmp_path / "raised_plate.xml"
    ET.ElementTree(root).write(path)
    env = SO101Env(cfg=SimConfig(images=False, randomization={"layout": {"enabled": False}}))
    env.task_impl.definition["scene"] = str(path)
    try:
        env.reset(seed=0)
        np.testing.assert_allclose(env.data.body("plate").xpos, [0.14, 0.13, 0.01])
        address = env.model.joint("red_joint").qposadr[0]
        env.data.qpos[address : address + 3] = [0.14, 0.13, 0.0245]
        env.data.qpos[address + 3 : address + 7] = [1, 0, 0, 0]
        env.mj.mj_forward(env.model, env.data)
        assert env.in_plate("red")
    finally:
        env.close()
