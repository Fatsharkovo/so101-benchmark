"""Load editable MJCF sources and derive task geometry from their actual contents."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .config import SimConfig
from .tasks.base import ObjectSpec, SceneSpec


def scene_path(definition: dict) -> Path | None:
    value = definition.get("scene")
    if not value:
        return None
    path = Path(value)
    if path.is_absolute():
        return path
    local = Path(definition["source"]).parent / path
    if local.exists():
        return local
    import task_scenes

    return Path(task_scenes.__file__).parent / path


def read_xml(path: Path) -> ET.Element:
    """Expand includes with paths resolved relative to the containing XML file."""

    def expand(source: Path, ancestors: set[Path]) -> ET.Element:
        source = source.resolve()
        if source in ancestors:
            raise ValueError(f"Recursive MJCF include: {source}")
        root = ET.parse(source).getroot()
        compiler = root.find("compiler")
        if compiler is not None and compiler.get("meshdir"):
            meshes = (source.parent / compiler.get("meshdir")).resolve()
            if not meshes.is_dir() and source.name == "common.xml":
                from .scene import ASSET_DIR

                meshes = ASSET_DIR / "assets"
            compiler.set("meshdir", str(meshes))

        def visit(parent):
            for child in list(parent):
                if child.tag == "include":
                    included = expand(source.parent / child.attrib["file"], ancestors | {source})
                    index = list(parent).index(child)
                    parent.remove(child)
                    for offset, element in enumerate(included):
                        parent.insert(index + offset, element)
                else:
                    visit(child)

        visit(root)
        return root

    root = expand(path, set())
    # Includes may each declare asset/worldbody sections; merge before applying edits.
    for tag in ("asset", "worldbody", "default", "contact", "equality", "actuator", "sensor", "custom"):
        sections = root.findall(tag)
        for extra in sections[1:]:
            sections[0].extend(extra)
            root.remove(extra)
    return root


def orientation(element: ET.Element) -> Rotation:
    if "quat" in element.attrib:
        q = np.fromstring(element.get("quat"), sep=" ")
        return Rotation.from_quat([*q[1:], q[0]])
    if "xyaxes" in element.attrib:
        x, y = np.fromstring(element.get("xyaxes"), sep=" ").reshape(2, 3)
        x /= np.linalg.norm(x)
        y -= np.dot(x, y) * x
        y /= np.linalg.norm(y)
        return Rotation.from_matrix(np.column_stack([x, y, np.cross(x, y)]))
    if "axisangle" in element.attrib:
        axis = np.fromstring(element.get("axisangle"), sep=" ")
        return Rotation.from_rotvec(axis[:3] / np.linalg.norm(axis[:3]) * axis[3])
    if "zaxis" in element.attrib:
        z = np.fromstring(element.get("zaxis"), sep=" ")
        return Rotation.align_vectors([z / np.linalg.norm(z)], [[0, 0, 1]])[0]
    return Rotation.from_euler("xyz", np.fromstring(element.get("euler", "0 0 0"), sep=" "))


def derive_spec(root: ET.Element, spec: SceneSpec) -> None:
    """Refresh scoring dimensions from named cube geoms and the plate boundary mesh."""
    world = root.find("worldbody")
    objects = []
    for body in world.findall("body"):
        name = body.get("name")
        geom = body.find(f"geom[@name='{name}_geom']")
        if body.find("freejoint") is None or geom is None:
            continue
        size = np.fromstring(geom.get("size"), sep=" ")
        if geom.get("type") != "box" or not np.allclose(size, size[0]):
            raise ValueError(f"Task object {name} must be a cube")
        objects.append(
            ObjectSpec(
                name,
                tuple(np.fromstring(geom.get("rgba"), sep=" ")[:3]),
                tuple(np.fromstring(body.get("pos"), sep=" ")),
                float(size[0] * 2),
                float(geom.get("mass")),
            )
        )
    spec.objects = objects
    plate = world.find("body[@name='plate']")
    spec.plate_xy = None
    if plate is None:
        return
    spec.plate_xy = tuple(np.fromstring(plate.get("pos"), sep=" ")[:2])
    bottom = plate.find("geom[@name='plate_bottom']")
    if bottom.get("type") == "mesh":
        mesh = root.find(f"asset/mesh[@name='{bottom.get('mesh')}']")
        vertices = np.fromstring(mesh.get("vertex"), sep=" ").reshape(-1, 3)
        spec.plate_shape = "rounded_square"
        spec.plate_radius = float(np.max(np.abs(vertices[:, :2])))
        # The first outline point is the rightmost point of the top-right corner arc.
        spec.plate_corner_radius = spec.plate_radius - float(vertices[0, 1])
        spec.plate_base_thickness = float(np.ptp(vertices[:, 2]))
    else:
        sizes = np.fromstring(bottom.get("size"), sep=" ")
        spec.plate_shape = "circle"
        spec.plate_radius, spec.plate_base_thickness = float(sizes[0]), float(sizes[1] * 2)
    rim = plate.find("geom[@name='plate_rim_0']")
    sizes = np.fromstring(rim.get("size"), sep=" ")
    spec.plate_wall_thickness, spec.plate_rim_height = float(sizes[1] * 2), float(sizes[2] * 2)
    spec.__post_init__()


def load_scene(path: Path, spec: SceneSpec, params: dict, cfg: SimConfig) -> ET.Element:
    from .scene import add_plate, numbers

    root = read_xml(path)
    derive_spec(root, spec)
    world = root.find("worldbody")
    if "colors" in params:
        for obj in spec.objects:
            if obj.name not in params["colors"]:
                world.remove(world.find(f"body[@name='{obj.name}']"))
        derive_spec(root, spec)
    missing = {params[key] for key in ("target", "base") if key in params} - {o.name for o in spec.objects}
    if missing:
        raise ValueError(f"Task objects missing from XML scene: {sorted(missing)}")
    for obj in spec.objects:
        body = world.find(f"body[@name='{obj.name}']")
        geom = body.find(f"geom[@name='{obj.name}_geom']")
        if obj.name in params.get("positions", {}):
            body.set("pos", numbers([*params["positions"][obj.name], obj.position[2]]))
        if "cube_size" in params:
            geom.set("size", numbers([params["cube_size"] / 2] * 3))
            position = np.fromstring(body.get("pos"), sep=" ")
            position[2] += (params["cube_size"] - obj.size) / 2
            body.set("pos", numbers(position))
        if "cube_mass" in params:
            geom.set("mass", str(params["cube_mass"]))
    plate_keys = [name for name in vars(spec) if name.startswith("plate_")]
    if spec.plate_xy is not None and any(key in params for key in plate_keys):
        for key in plate_keys:
            if key in params:
                setattr(spec, key, params[key])
        spec.__post_init__()
        world.remove(world.find("body[@name='plate']"))
        asset = root.find("asset")
        old_mesh = asset.find("mesh[@name='plate_base_mesh']")
        if old_mesh is not None:
            asset.remove(old_mesh)
        add_plate(world, asset, spec, spec.plate_xy)
    derive_spec(root, spec)
    root.find("option").set("timestep", str(1 / cfg.physics_hz))
    visual = root.find("visual/global")
    visual.set("offwidth", str(cfg.width))
    visual.set("offheight", str(cfg.height))
    return root
