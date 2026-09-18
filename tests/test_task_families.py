"""Scene-independent task selection, catalog extension, and legacy compatibility."""

import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

import task_scenes
from lerobot_env_so101.cli import main
from lerobot_env_so101.config import COLORS, SimConfig, load_config
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.tasks.base import discover, make_task
from lerobot_env_so101.tasks.manipulation import StackBlueOnRed, StackCubes
from lerobot_env_so101.teleop import TeleopSession
from lerobot_env_so101.xml_scene import read_xml, scene_path


def family():
    return {
        "family": "extra_stacks",
        "class": "lerobot_env_so101.tasks.manipulation:StackCubes",
        "scene": "stack_cubes.xml",
        "instruction_template": "Put {target} on {base}.",
        "params": {"stable_seconds": 1.0, "positions": {"red": [0.19, 0.045]}},
        "tasks": {
            "extra_yellow_red": {"params": {"target": "yellow", "base": "red"}},
            "extra_red_yellow": {
                "params": {"target": "red", "base": "yellow", "stable_seconds": 0.8},
                "instruction": "Stack red on yellow.",
            },
        },
    }


def write_family(tmp_path, document):
    path = tmp_path / "family.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def test_catalog_expands_ten_tasks_in_two_families():
    registry = discover()
    assert len(registry) == 10
    assert list(registry) == sorted(registry)
    assert {d["family"] for d in registry.values()} == {"place_in_plate", "stack_cubes"}
    assert {d["scene"] for d in registry.values()} == {"place_in_plate.xml", "stack_cubes.xml"}
    assert StackBlueOnRed is StackCubes
    for definition in registry.values():
        assert definition["params"]["target"] in definition["instruction"]


def test_family_inheritance_and_task_instances_are_independent(tmp_path):
    write_family(tmp_path, family())
    registry = discover([str(tmp_path)])
    a, b = registry["extra_yellow_red"], registry["extra_red_yellow"]
    assert a["instruction"] == "Put yellow on red."
    assert b["instruction"] == "Stack red on yellow."
    assert a["params"]["stable_seconds"] == 1
    assert b["params"]["stable_seconds"] == 0.8
    a["params"]["positions"]["red"][0] = 999
    assert b["params"]["positions"]["red"][0] == 0.19
    first = StackCubes(b)
    first.params["positions"]["red"][0] = 888
    assert StackCubes(b).params["positions"]["red"][0] == 0.19


def test_added_yaml_task_uses_relative_scene_without_code_changes(tmp_path):
    document = family()
    document["scene"] = "local_scene.xml"
    document["params"] = {"stable_seconds": 1.0}
    source = read_xml(Path(task_scenes.__file__).parent / "stack_cubes.xml")
    # Geometry defaults come from the XML, including when Task.scene() is used directly.
    source.find("worldbody/body[@name='slot_0']").set("pos", "0.18 -0.045 0.0135")
    ET.ElementTree(source).write(tmp_path / "local_scene.xml")
    write_family(tmp_path, document)
    env = SO101Env(
        "extra_yellow_red",
        SimConfig(images=False, episode_seconds=20, randomization={"layout": {"enabled": False}}),
        [str(tmp_path)],
    )
    try:
        spec = env.task_impl.scene()
        assert spec.objects[0].name == "red"
        assert spec.objects[0].position[0] == 0.18
        env.reset(seed=0)
        assert set(env.object_sizes) == {"yellow", "red"}
        oracle = env.task_impl.make_oracle(env)
        for _ in range(env._max_episode_steps):
            _, _, done, truncated, info = env.step(oracle.action())
            if done or truncated:
                break
        assert info["is_success"]
    finally:
        env.close()


@pytest.mark.parametrize("seed", [0, 17])
def test_plate_scene_is_identical_across_targets(seed):
    reference = None
    for color in COLORS:
        env = SO101Env(f"place_{color}_in_plate", load_config("configs/teleop.yaml").sim)
        env.cfg.images = False
        try:
            env.reset(seed=seed)
            current = (env.model_xml, env.sampled, env.snapshot())
            if reference is None:
                reference = current
            assert current == reference
            assert env.task_impl.stable_objects == [color, "plate"]
        finally:
            env.close()


def test_stack_roles_do_not_change_layout_and_reset_still_randomizes():
    layouts = []
    for seed in (0, 17):
        states = []
        for task in ("stack_blue_on_red", "stack_red_on_blue"):
            cfg = load_config("configs/teleop.yaml").sim
            cfg.images = False
            env = SO101Env(task, cfg)
            try:
                env.reset(seed=seed)
                states.append((env.model_xml, env.sampled, env.snapshot()))
                assert [o.name for o in env.scene_spec.objects] == ["blue", "red"]
                initial = env.snapshot()
                for _ in range(3):
                    env.step(np.array([20, -60, 70, 60, 0, 70]))
                env.restore_initial()
                restored = env.snapshot()
                for key in ("qpos", "qvel", "ctrl", "time"):
                    np.testing.assert_array_equal(restored[key], initial[key])
                for name in env.object_sizes:
                    np.testing.assert_allclose(restored["objects"][name], initial["objects"][name], atol=1e-8)
            finally:
                env.close()
        assert states[0] == states[1]
        layouts.append(states[0][1]["objects"])
    assert layouts[0] != layouts[1]


@pytest.mark.parametrize("task", [name for name in discover() if name.startswith("stack_")])
def test_stack_template_binds_only_selected_colors(task):
    env = SO101Env(task, SimConfig(images=False))
    try:
        env.reset(seed=0)
        params = env.task_impl.params
        assert set(env.object_sizes) == {params["target"], params["base"]}
        assert [obj.name for obj in env.scene_spec.objects] == sorted(env.object_sizes)
        for name in env.object_sizes:
            np.testing.assert_allclose(env.model.geom(f"{name}_geom").rgba, [*COLORS[name], 1])
        assert "slot_0" not in env.model_xml
    finally:
        env.close()


@pytest.mark.parametrize(
    "change,match",
    [
        ({"tasks": {}}, "nonempty"),
        ({"params": []}, "mapping"),
        ({"instruction_template": "Missing {unknown}."}, "unknown"),
        ({"tasks": {"bad": {"class": "override"}}}, "only params and instruction"),
    ],
)
def test_invalid_family_reports_source(tmp_path, change, match):
    document = family()
    document.update(change)
    path = write_family(tmp_path, document)
    with pytest.raises(ValueError, match=match) as error:
        discover([str(tmp_path)])
    assert str(path) in str(error.value)


@pytest.mark.parametrize("params", [{"target": "purple", "base": "red"}, {"target": "red", "base": "red"}])
def test_invalid_roles_report_task_and_source(tmp_path, params):
    document = family()
    document["tasks"] = {"invalid_stack": {"params": params}}
    path = write_family(tmp_path, document)
    with pytest.raises(ValueError) as error:
        make_task("invalid_stack", [str(tmp_path)])
    assert "invalid_stack" in str(error.value) and str(path) in str(error.value)


def test_duplicate_ids_rejected_within_and_across_files(tmp_path):
    document = family()
    path = write_family(tmp_path, document)
    path.write_text(path.read_text() + "  extra_yellow_red: {}\n")
    with pytest.raises(ValueError, match="Duplicate YAML key"):
        discover([str(tmp_path)])
    write_family(tmp_path, document)
    (tmp_path / "duplicate.yaml").write_text(path.read_text())
    with pytest.raises(ValueError, match="Duplicate task id"):
        discover([str(tmp_path)])


def test_family_yaml_anchors_and_explicit_override_are_preserved(tmp_path):
    (tmp_path / "anchors.yaml").write_text("""
family: anchored
class: lerobot_env_so101.tasks.manipulation:StackCubes
scene: stack_cubes.xml
instruction_template: "Put {target} on {base}."
params: &defaults
  target: blue
  base: red
tasks:
  anchored_stack:
    params:
      <<: *defaults
      target: yellow
""")
    task = make_task("anchored_stack", [str(tmp_path)])
    assert task.params == {"target": "yellow", "base": "red"}
    assert task.instruction == "Put yellow on red."


def test_missing_template_object_reports_scene(tmp_path):
    root = read_xml(Path(task_scenes.__file__).parent / "stack_cubes.xml")
    world = root.find("worldbody")
    world.remove(world.find("body[@name='slot_1']"))
    path = tmp_path / "missing.xml"
    ET.ElementTree(root).write(path)
    env = SO101Env("stack_blue_on_red", SimConfig(images=False))
    env.task_impl.definition["scene"] = str(path)
    try:
        with pytest.raises(ValueError, match="slot_1") as error:
            env.reset(seed=0)
        assert str(path) in str(error.value)
    finally:
        env.close()


def test_legacy_task_yaml_and_scene_names_still_load(tmp_path):
    definition = {
        "id": "legacy_stack",
        "class": "lerobot_env_so101.tasks.manipulation:StackBlueOnRed",
        "scene": "stack_blue_on_red.xml",
        "instruction": "Stack blue on red.",
        "params": {"target": "blue", "base": "red"},
    }
    (tmp_path / "legacy.yaml").write_text(yaml.safe_dump(definition))
    env = SO101Env("legacy_stack", SimConfig(images=False), [str(tmp_path)])
    try:
        env.reset(seed=0)
        assert set(env.object_sizes) == {"blue", "red"}
        assert env.task_impl.stable_objects == ["blue", "red"]
        assert scene_path(env.task_impl.definition).name == "stack_cubes.xml"
        for color in COLORS:
            root = read_xml(Path(task_scenes.__file__).parent / f"place_{color}_in_plate.xml")
            assert root.find("worldbody/body[@name='plate']") is not None
    finally:
        env.close()


def test_captured_stack_scene_ignores_current_template_and_preserves_joint_order(tmp_path):
    env = SO101Env("stack_blue_on_red", SimConfig(images=False))
    try:
        env.reset(seed=0)
        root = ET.fromstring(env.model_xml)
        world = root.find("worldbody")
        # Old recordings can contain arbitrary ordering and transforms.
        blue = world.find("body[@name='blue']")
        world.remove(blue)
        world.append(blue)
        path = tmp_path / "captured.xml"
        ET.ElementTree(root).write(path)
        env.task_impl.definition["scene"] = str(tmp_path / "removed.xml")
        env.reset(seed=0, options={"scene_xml": path})
        assert env.model.joint("red_joint").qposadr[0] < env.model.joint("blue_joint").qposadr[0]
        assert set(env.object_sizes) == {"blue", "red"}
    finally:
        env.close()


def test_grouped_and_legacy_list(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["so101-bench", "list"])
    main()
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 10
    assert lines[0].startswith("place_blue_in_plate:")
    monkeypatch.setattr(sys, "argv", ["so101-bench", "list", "--group-by-family"])
    main()
    output = capsys.readouterr().out
    assert "place_in_plate:\n" in output and "stack_cubes:\n" in output
    assert "  stack_green_on_yellow:" in output and "  stack_yellow_on_blue:" in output


def test_teleop_records_resolved_family_task_and_instruction():
    class Writer:
        def __init__(self):
            self.messages = []

        def submit(self, command, payload=None):
            self.messages.append((command, copy.deepcopy(payload)))

    cfg = load_config(overrides={"video": False, "sim": {"images": False}})
    env = SO101Env("stack_green_on_yellow", cfg.sim)
    try:
        env.reset(seed=0)
        writer = Writer()
        session = TeleopSession(env, writer, cfg, 0)
        session.state = "WAITING"
        session.start()
        oracle = env.task_impl.make_oracle(env)
        for _ in range(env._max_episode_steps):
            session.step(oracle.action())
            if session.state == "SAVING":
                break
        assert session.state == "SAVING"
        command, metadata = writer.messages[-1]
        assert command == "save"
        task = metadata["task"]
        assert task["family"] == "stack_cubes"
        assert task["id"] == "stack_green_on_yellow"
        assert task["params"]["target"] == "green" and task["params"]["base"] == "yellow"
        assert task["instruction"] == writer.messages[0][1]["task"]
        assert "slot_0" not in metadata["scene_xml"]
        json.dumps(task)
    finally:
        env.close()
