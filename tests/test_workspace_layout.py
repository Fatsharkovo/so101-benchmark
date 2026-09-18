"""Global workspace placement, reachability and closed-gripper reset regressions."""

import copy
import itertools

import numpy as np
import pytest

from lerobot_env_so101.config import SimConfig, load_config
from lerobot_env_so101.env import SO101Env
from lerobot_env_so101.layout import sample_workspace
from lerobot_env_so101.oracle import solve_ik
from lerobot_env_so101.scene import make_xml
from lerobot_env_so101.tasks.base import make_task
from lerobot_env_so101.xml_scene import scene_path


@pytest.mark.parametrize(
    "task_id",
    [
        "place_blue_in_plate",
        "stack_blue_on_red",
        "stack_orange_on_green",
        "stack_red_on_blue",
        "stack_green_on_yellow",
        "stack_yellow_on_blue",
    ],
)
def test_all_objects_cover_workspace_with_clearance_and_repeatable_seeds(task_id):
    cfg = load_config("configs/teleop.yaml").sim
    task = make_task(task_id)
    spec = task.scene()
    layouts = []
    for seed in range(128):
        _, layout = make_xml(spec, cfg, seed, scene_path(task.definition), task.params)
        layouts.append(layout)
        xy = {name: np.array(obj["position"][:2]) for name, obj in layout["objects"].items()}
        for a, b in itertools.combinations(xy, 2):
            radii = (layout["objects"][a]["size"] + layout["objects"][b]["size"]) / np.sqrt(2)
            assert np.linalg.norm(xy[a] - xy[b]) >= radii + 0.025
        if "plate_xy" in layout:
            for name, position in xy.items():
                radius = layout["objects"][name]["size"] / np.sqrt(2)
                assert spec.plate_distance(position - layout["plate_xy"]) >= radius + 0.025
            xy["plate"] = np.array(layout["plate_xy"])
        for name, position in xy.items():
            lateral_limit = 0.18 if name == "plate" else 0.15
            assert 0.12 <= position[0] <= 0.27
            assert -lateral_limit <= position[1] <= lateral_limit
            assert 0.19 <= np.linalg.norm(position) <= 0.29
        if seed:
            for name in layout["objects"]:
                assert layout["objects"][name]["position"] != layouts[-2]["objects"][name]["position"]
            if "plate_xy" in layout:
                assert layout["plate_xy"] != layouts[-2]["plate_xy"]
    # Every color, and the plate, can move between both sides and across depth.
    tracks = [[s["objects"][name]["position"][:2] for s in layouts] for name in layouts[0]["objects"]]
    if "plate_xy" in layouts[0]:
        tracks.append([s["plate_xy"] for s in layouts])
    for track in tracks:
        points = np.asarray(track)
        assert np.ptp(points[:, 0]) > 0.12
        assert points[:, 1].min() < -0.14
        assert points[:, 1].max() > 0.14
    _, repeated = make_xml(spec, cfg, 0, scene_path(task.definition), task.params)
    assert repeated == layouts[0]


def test_workspace_does_not_use_xml_xy_anchors_and_rejects_impossible_packing():
    spec = make_task("place_blue_in_plate").scene()
    moved = copy.deepcopy(spec)
    for obj in moved.objects:
        obj.position = (0.01, 0.01, obj.position[2])
    moved.plate_xy = (0.01, 0.01)
    sizes = {obj.name: obj.size for obj in spec.objects}
    a, plate_a = sample_workspace(spec, sizes, np.random.default_rng(11))
    b, plate_b = sample_workspace(moved, sizes, np.random.default_rng(11))
    for name in a:
        np.testing.assert_array_equal(a[name], b[name])
    np.testing.assert_array_equal(plate_a, plate_b)
    with pytest.raises(ValueError, match="Cannot fit all objects"):
        sample_workspace(spec, dict.fromkeys(sizes, 0.5), np.random.default_rng(0))


def test_workspace_grid_has_downward_grasp_and_approach_without_robot_collisions():
    env = SO101Env("stack_blue_on_red", SimConfig(images=False))
    try:
        env.reset(seed=0)
        data = env.mj.MjData(env.model)
        table = env.model.body("table").id
        for x, y in itertools.product(np.linspace(0.12, 0.27, 9), np.linspace(-0.18, 0.18, 19)):
            if not 0.19 <= np.hypot(x, y) <= 0.29:
                continue
            initial = None
            # Include approach above a two-cube stack, low grasp and placement.
            for z in (0.115, 0.09, 0.041, 0.0145):
                position = np.array([x, y, z])
                initial = solve_ik(env, position, initial)
                data.qpos[:] = env.data.qpos
                data.qpos[env.qadr[:5]] = initial
                data.qpos[env.qadr[5]] = env.to_sim([0, 0, 0, 0, 0, 70])[5]
                env.mj.mj_forward(env.model, data)
                assert np.linalg.norm(data.site("gripperframe").xpos - position) < 0.003
                for contact in data.contact:
                    bodies = env.model.geom_bodyid[[contact.geom1, contact.geom2]]
                    if any(b in env.robot_bodies for b in bodies) and all(
                        b in env.robot_bodies or b == table for b in bodies
                    ):
                        assert contact.dist >= -0.0002, (position, bodies, contact.dist)
    finally:
        env.close()


@pytest.mark.parametrize("task_id", ["place_blue_in_plate", "stack_green_on_yellow"])
def test_initial_reset_and_restore_start_with_closed_gripper(task_id, monkeypatch):
    cfg = load_config("configs/teleop.yaml").sim
    cfg.images = False
    env = SO101Env(task_id, cfg)
    try:
        import mujoco

        original_step = mujoco.mj_step
        first_states = []

        def step(model, data):
            if data.time == 0:
                first_states.append((data.qpos[env.qadr[5]], data.ctrl[env.actuator_ids[5]]))
            original_step(model, data)

        monkeypatch.setattr(mujoco, "mj_step", step)
        for seed in (0, 1):
            env.reset(seed=seed)
            np.testing.assert_allclose(first_states[-1], env.limits[5, 0], atol=1e-7)
            assert env.applied_action[5] == 0
            assert env.observe()["agent_pos"][5] < 2
            initial = env.data.qpos.copy()
            action = env.applied_action.copy()
            action[5] = 70
            for _ in range(10):
                env.step(action, trial=True)
            assert env.observe()["agent_pos"][5] > 60
            env.restore_initial()
            np.testing.assert_allclose(env.data.qpos, initial, atol=1e-12)
            assert env.applied_action[5] == 0
    finally:
        env.close()


def test_invalid_layout_mode_is_rejected():
    with pytest.raises(ValueError, match="layout.mode"):
        SimConfig(randomization={"layout": {"enabled": True, "mode": "typo"}})
