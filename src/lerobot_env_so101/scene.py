from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .config import SimConfig
from .tasks.base import SceneSpec

ASSET_DIR = Path(__file__).resolve().parent / "assets/so101"


def numbers(values) -> str:
    return " ".join(str(float(v)) for v in values)


def look_at(position, target) -> str:
    z = np.asarray(position) - np.asarray(target)
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return numbers([*x, *y])


def add_plate(world: ET.Element, asset: ET.Element, spec: SceneSpec, plate_xy: np.ndarray) -> None:
    """Build a thin light-blue plate with a rounded boundary shared by scoring."""
    plate = ET.SubElement(world, "body", name="plate", pos=numbers([*plate_xy, 0]))
    half, radius = spec.plate_radius, spec.plate_corner_radius
    base = spec.plate_base_thickness
    wall = spec.plate_wall_thickness
    rim_height = spec.plate_rim_height

    def bottom(name: str, kind: str, size: list[float], xy=(0, 0)) -> None:
        ET.SubElement(
            plate,
            "geom",
            name=name,
            type=kind,
            size=numbers(size),
            pos=numbers([*xy, base / 2]),
            rgba="0.48 0.76 0.9 1",
            friction="0.8 0.005 0.0001",
        )

    if spec.plate_shape == "circle":
        bottom("plate_bottom", "cylinder", [half, base / 2])
        angles = np.linspace(0, 2 * math.pi, 65)[:-1]
        boundary = np.column_stack([np.cos(angles), np.sin(angles)]) * (half - wall / 2)
    else:
        inset = half - radius
        boundary = []
        outline = []
        for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
            center = np.array([sx, sy]) * inset
            angles = np.linspace(i * math.pi / 2, (i + 1) * math.pi / 2, 9)
            radial = np.column_stack([np.cos(angles), np.sin(angles)])
            boundary.extend(center + radial * (radius - wall / 2))
            outline.extend(center + radial * radius)
        boundary = np.asarray(boundary)
        # A single convex base avoids overlapping contact surfaces and coplanar rendering flicker.
        vertices = [[*point, z] for z in (0, base) for point in outline]
        ET.SubElement(asset, "mesh", name="plate_base_mesh", vertex=numbers(np.ravel(vertices)))
        ET.SubElement(
            plate,
            "geom",
            name="plate_bottom",
            type="mesh",
            mesh="plate_base_mesh",
            rgba="0.48 0.76 0.9 1",
            friction="0.8 0.005 0.0001",
        )
    for i, start in enumerate(boundary):
        end = boundary[(i + 1) % len(boundary)]
        delta = end - start
        ET.SubElement(
            plate,
            "geom",
            type="box",
            name=f"plate_rim_{i}",
            pos=numbers([*((start + end) / 2), base + rim_height / 2]),
            size=numbers([np.linalg.norm(delta) / 2 + 0.00005, wall / 2, rim_height / 2]),
            euler=f"0 0 {math.atan2(delta[1], delta[0])}",
            rgba="0.4 0.68 0.84 1",
        )


def _base_xml(spec: SceneSpec, cfg: SimConfig) -> ET.Element:
    root = ET.parse(ASSET_DIR / "so101.xml").getroot()
    root.find("compiler").set("meshdir", str(ASSET_DIR / "assets"))
    root.find("option").set("timestep", str(1 / cfg.physics_hz))
    root.find("option").set("iterations", "50")
    ET.SubElement(root.find("option"), "flag", multiccd="enable")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(cfg.width), offheight=str(cfg.height))
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.6 0.6 0.6")
    asset = root.find("asset")
    ET.SubElement(
        asset,
        "texture",
        name="daylight",
        type="skybox",
        builtin="gradient",
        rgb1="0.55 0.72 0.88",
        rgb2="0.92 0.95 0.98",
        width="512",
        height="3072",
    )
    ET.SubElement(
        asset,
        "texture",
        name="ground_grid",
        type="2d",
        builtin="checker",
        rgb1="0.72 0.75 0.78",
        rgb2="0.79 0.82 0.85",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "material",
        name="ground_material",
        texture="ground_grid",
        texrepeat="2 2",
        texuniform="true",
        reflectance="0",
    )
    world = root.find("worldbody")
    # Infinite visual ground below the tabletop. Falling objects still cross the failure threshold.
    ET.SubElement(
        world,
        "geom",
        name="ground",
        type="plane",
        pos="0 0 -0.08",
        size="0 0 0.01",
        material="ground_material",
        contype="0",
        conaffinity="0",
    )
    for item in ET.fromstring(f"<root>{spec.assets_xml}</root>"):
        root.find("asset").append(item)
    # Named camera target is stable across tasks.
    ET.SubElement(world, "light", name="key_light", pos="0.2 -0.3 0.9", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    table = ET.SubElement(world, "body", name="table", pos="0.1 0 -0.025")
    ET.SubElement(
        table,
        "geom",
        name="table_top",
        type="box",
        size="0.42 0.38 0.025",
        rgba="0.65 0.65 0.65 1",
        friction="0.8 0.005 0.0001",
    )
    for name, pos in (("front", (0.45, 0, 0.35)), ("overview", (0.60, -0.65, 0.65))):
        camera_cfg = cfg.cameras.get(name, {})
        position = camera_cfg.get("position", pos)
        # Front optical axis points down 60 degrees from the horizontal.
        target = camera_cfg.get(
            "target", (0.45 - 0.35 / math.sqrt(3), 0, 0) if name == "front" else (0.16, 0, 0.06)
        )
        ET.SubElement(
            world,
            "camera",
            name=name,
            pos=numbers(position),
            xyaxes=look_at(position, target),
            fovy=str(camera_cfg.get("fovy", 60 if name == "front" else 48)),
        )
    wrist = root.find(".//camera[@name='wrist_cam']")
    wrist.set("name", "wrist")
    for attr in ("resolution", "sensorsize", "focal"):
        wrist.attrib.pop(attr, None)
    wrist.set("fovy", str(cfg.cameras.get("wrist", {}).get("fovy", 65)))
    for attr in ("pos", "euler"):
        if attr in cfg.cameras.get("wrist", {}):
            wrist.set(attr, numbers(cfg.cameras["wrist"][attr]))
    return root


def make_xml(
    spec: SceneSpec,
    cfg: SimConfig,
    seed: int,
    scene_path: Path | None = None,
    params: dict | None = None,
) -> tuple[str, dict]:
    from .xml_scene import load_scene

    root = load_scene(scene_path, spec, params or {}, cfg) if scene_path else _base_xml(spec, cfg)
    world, asset = root.find("worldbody"), root.find("asset")
    sampled = {"seed": seed, "objects": {}, "groups": {}}
    streams = {
        key: np.random.default_rng(np.random.SeedSequence([seed, i]))
        for i, key in enumerate(("layout", "appearance", "camera", "size", "physics"))
    }
    scales = {}
    for key in ("size", "physics"):
        group = cfg.randomization.get(key, {})
        scales[key] = group if group.get("enabled", False) else {}
    sizes = {
        obj.name: obj.size * streams["size"].uniform(*scales["size"].get("scale", [1, 1]))
        for obj in spec.objects
    }
    layout = cfg.randomization.get("layout", {})
    rng = streams["layout"]
    for attempt in range(200):
        positions = {}
        for obj in spec.objects:
            p = np.array(obj.position, dtype=float)
            if layout.get("enabled", False):
                p[:2] += rng.uniform(
                    -layout.get("position_jitter", 0.005), layout.get("position_jitter", 0.005), 2
                )
            p[2] += (sizes[obj.name] - obj.size) / 2
            positions[obj.name] = p
        plate_xy = np.array(spec.plate_xy) if spec.plate_xy is not None else None
        if plate_xy is not None and layout.get("enabled", False):
            plate_xy += rng.uniform(
                -layout.get("position_jitter", 0.005), layout.get("position_jitter", 0.005), 2
            )
        valid = all(0.10 < np.linalg.norm(p[:2]) < 0.31 for p in positions.values())
        if plate_xy is not None:
            valid &= 0.13 < np.linalg.norm(plate_xy) < 0.29
        for i, a in enumerate(spec.objects):
            for b in spec.objects[i + 1 :]:
                valid &= (
                    np.linalg.norm(positions[a.name][:2] - positions[b.name][:2])
                    > (sizes[a.name] + sizes[b.name]) / 2 + 0.025
                )
            if plate_xy is not None:
                valid &= spec.plate_distance(positions[a.name][:2] - plate_xy) > sizes[a.name] + 0.01
        if valid:
            break
    else:
        raise ValueError(
            "Cannot sample collision-free reachable layout within 200 attempts; reduce randomization"
        )
    for obj in spec.objects:
        yaw = rng.uniform(*layout.get("yaw_deg", [-5, 5])) if layout.get("enabled", False) else 0
        size = sizes[obj.name]
        mass = obj.mass * streams["physics"].uniform(*scales["physics"].get("mass_scale", [1, 1]))
        friction = streams["physics"].uniform(*scales["physics"].get("friction_scale", [1, 1]))
        body = world.find(f"body[@name='{obj.name}']")
        if body is None:
            body = ET.SubElement(world, "body", name=obj.name)
            ET.SubElement(body, "freejoint", name=f"{obj.name}_joint")
            ET.SubElement(
                body,
                "geom",
                name=f"{obj.name}_geom",
                type="box",
                rgba=numbers([*obj.color, 1]),
                friction="1 0.005 0.0001",
                condim="4",
                solref="0.008 1",
            )
        body.set("pos", numbers(positions[obj.name]))
        if yaw:
            from .xml_scene import orientation

            rotation = Rotation.from_euler("z", yaw, degrees=True) * orientation(body)
            for key in ("quat", "euler", "xyaxes", "axisangle", "zaxis"):
                body.attrib.pop(key, None)
            q = rotation.as_quat()
            body.set("quat", numbers([q[3], *q[:3]]))
        geom = body.find(f"geom[@name='{obj.name}_geom']")
        geom.set("size", numbers([size / 2] * 3))
        geom.set("mass", str(mass))
        base_friction = np.fromstring(geom.get("friction", "1 0.005 0.0001"), sep=" ")
        geom.set("friction", numbers(base_friction * friction))
        sampled["objects"][obj.name] = {
            "position": positions[obj.name].tolist(),
            "yaw_deg": float(yaw),
            "size": size,
            "mass": mass,
            "friction": friction,
        }
    if plate_xy is not None:
        plate = world.find("body[@name='plate']")
        if plate is None:
            add_plate(world, asset, spec, plate_xy)
        else:
            position = np.fromstring(plate.get("pos", "0 0 0"), sep=" ")
            position[:2] = plate_xy
            plate.set("pos", numbers(position))
        sampled["plate_xy"] = plate_xy.tolist()
    for item in ET.fromstring(f"<root>{spec.worldbody_xml}</root>"):
        world.append(item)
    appearance = cfg.randomization.get("appearance", {})
    if appearance.get("enabled", False):
        brightness = streams["appearance"].uniform(*appearance.get("brightness", [0.8, 1.2]))
        gray = streams["appearance"].uniform(*appearance.get("table_gray", [0.35, 0.65]))
        root.find(".//light[@name='key_light']").set("diffuse", numbers([min(1, 0.8 * brightness)] * 3))
        root.find(".//geom[@name='table_top']").set("rgba", numbers([gray] * 3 + [1]))
        sampled["groups"]["appearance"] = {"brightness": brightness, "table_gray": gray}
    camera_group = cfg.randomization.get("camera", {})
    if camera_group.get("enabled", False):
        for camera in root.iter("camera"):
            if camera.get("name") == "overview":
                continue
            jitter = camera_group.get("position_jitter", 0.005)
            p = np.fromstring(camera.get("pos"), sep=" ") + streams["camera"].uniform(-jitter, jitter, 3)
            camera.set("pos", numbers(p))
            angle = camera_group.get("rotation_deg", 2.0)
            delta = Rotation.from_euler("xyz", streams["camera"].uniform(-angle, angle, 3), degrees=True)
            from .xml_scene import orientation

            rot = orientation(camera)
            for key in ("quat", "euler", "xyaxes", "axisangle", "zaxis"):
                camera.attrib.pop(key, None)
            quat = (rot * delta).as_quat()
            camera.set("quat", numbers([quat[3], *quat[:3]]))
            sampled["groups"][camera.get("name")] = dict(camera.attrib)
    # Upstream keyframes only contain robot DOFs; objects introduce free joints.
    for keyframe in root.findall("keyframe"):
        root.remove(keyframe)
    return ET.tostring(root, encoding="unicode"), sampled
