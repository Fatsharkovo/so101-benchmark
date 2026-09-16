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


def make_xml(spec: SceneSpec, cfg: SimConfig, seed: int) -> tuple[str, dict]:
    root = ET.parse(ASSET_DIR / "so101.xml").getroot()
    root.find("compiler").set("meshdir", str(ASSET_DIR / "assets"))
    root.find("option").set("timestep", str(1 / cfg.physics_hz))
    root.find("option").set("iterations", "50")
    ET.SubElement(root.find("option"), "flag", multiccd="enable")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(cfg.width), offheight=str(cfg.height))
    world = root.find("worldbody")
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
        rgba="0.5 0.5 0.5 1",
        friction="0.8 0.005 0.0001",
    )
    for name, pos in (("front", (0.65, -0.45, 0.45)), ("overview", (0.60, -0.65, 0.65))):
        camera_cfg = cfg.cameras.get(name, {})
        position = camera_cfg.get("position", pos)
        target = camera_cfg.get("target", (0.16, 0, 0.06))
        ET.SubElement(
            world,
            "camera",
            name=name,
            pos=numbers(position),
            xyaxes=look_at(position, target),
            fovy=str(camera_cfg.get("fovy", 48)),
        )
    wrist = root.find(".//camera[@name='wrist_cam']")
    wrist.set("name", "wrist")
    for attr in ("resolution", "sensorsize", "focal"):
        wrist.attrib.pop(attr, None)
    wrist.set("fovy", str(cfg.cameras.get("wrist", {}).get("fovy", 65)))
    for attr in ("pos", "euler"):
        if attr in cfg.cameras.get("wrist", {}):
            wrist.set(attr, numbers(cfg.cameras["wrist"][attr]))
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
                    -layout.get("position_jitter", 0.012), layout.get("position_jitter", 0.012), 2
                )
            p[2] = sizes[obj.name] / 2 + 0.001
            positions[obj.name] = p
        plate_xy = np.array(spec.plate_xy) if spec.plate_xy is not None else None
        if plate_xy is not None and layout.get("enabled", False):
            plate_xy += rng.uniform(
                -layout.get("position_jitter", 0.012), layout.get("position_jitter", 0.012), 2
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
                valid &= (
                    np.linalg.norm(positions[a.name][:2] - plate_xy)
                    > spec.plate_radius + sizes[a.name] + 0.01
                )
        if valid:
            break
    else:
        raise ValueError(
            "Cannot sample collision-free reachable layout within 200 attempts; reduce randomization"
        )
    for obj in spec.objects:
        yaw = rng.uniform(*layout.get("yaw_deg", [-10, 10])) if layout.get("enabled", False) else 0
        size = sizes[obj.name]
        mass = obj.mass * streams["physics"].uniform(*scales["physics"].get("mass_scale", [1, 1]))
        friction = streams["physics"].uniform(*scales["physics"].get("friction_scale", [1, 1]))
        body = ET.SubElement(
            world,
            "body",
            name=obj.name,
            pos=numbers(positions[obj.name]),
            euler=numbers([0, 0, np.deg2rad(yaw)]),
        )
        ET.SubElement(body, "freejoint", name=f"{obj.name}_joint")
        ET.SubElement(
            body,
            "geom",
            name=f"{obj.name}_geom",
            type="box",
            size=numbers([size / 2] * 3),
            rgba=numbers([*obj.color, 1]),
            mass=str(mass),
            friction=f"{friction} 0.005 0.0001",
            condim="4",
            solref="0.008 1",
        )
        sampled["objects"][obj.name] = {
            "position": positions[obj.name].tolist(),
            "yaw_deg": float(yaw),
            "size": size,
            "mass": mass,
            "friction": friction,
        }
    if plate_xy is not None:
        plate = ET.SubElement(world, "body", name="plate", pos=numbers([*plate_xy, 0]))
        ET.SubElement(
            plate,
            "geom",
            name="plate_bottom",
            type="cylinder",
            size=f"{spec.plate_radius} 0.003",
            pos="0 0 0.003",
            rgba="0.92 0.92 0.9 1",
            friction="0.8 0.005 0.0001",
        )
        for i in range(32):
            a = i * 2 * math.pi / 32
            r = spec.plate_radius
            ET.SubElement(
                plate,
                "geom",
                type="box",
                name=f"plate_rim_{i}",
                pos=numbers([r * math.cos(a), r * math.sin(a), 0.009]),
                size=numbers([0.003, r * math.tan(math.pi / 32) + 0.0005, 0.006]),
                euler=f"0 0 {a}",
                rgba="0.85 0.85 0.83 1",
            )
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
            if "xyaxes" in camera.attrib:
                xy = np.fromstring(camera.attrib.pop("xyaxes"), sep=" ").reshape(2, 3)
                rot = Rotation.from_matrix(np.column_stack([xy[0], xy[1], np.cross(*xy)]))
            else:
                rot = Rotation.from_euler("xyz", np.fromstring(camera.attrib.pop("euler", "0 0 0"), sep=" "))
            quat = (rot * delta).as_quat()
            camera.set("quat", numbers([quat[3], *quat[:3]]))
            sampled["groups"][camera.get("name")] = dict(camera.attrib)
    # Upstream keyframes only contain robot DOFs; objects introduce free joints.
    for keyframe in root.findall("keyframe"):
        root.remove(keyframe)
    return ET.tostring(root, encoding="unicode"), sampled
