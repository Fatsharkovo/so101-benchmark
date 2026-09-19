from __future__ import annotations

import copy
import importlib
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ObjectSpec:
    name: str
    color: tuple[float, float, float]
    position: tuple[float, float, float]
    size: float = 0.025
    mass: float = 0.010

    def __post_init__(self):
        if self.size <= 0 or self.mass <= 0 or len(self.position) != 3:
            raise ValueError("Objects require positive size/mass and a three-dimensional position")


@dataclass
class SceneSpec:
    objects: list[ObjectSpec]
    plate_xy: tuple[float, float] | None = None
    # Radius for circles; half the outer side length for rounded squares.
    plate_radius: float = 0.050
    # Additional MJCF worldbody elements for custom tasks.
    worldbody_xml: str = ""
    assets_xml: str = ""
    plate_shape: str = "rounded_square"
    plate_corner_radius: float = 0.012
    plate_base_thickness: float = 0.002
    plate_wall_thickness: float = 0.002
    plate_rim_height: float = 0.010
    plate_mass: float = 0.050
    plate_movable: bool = True

    def __post_init__(self):
        if self.plate_radius <= 0 or len({o.name for o in self.objects}) != len(self.objects):
            raise ValueError("Scene requires a positive plate radius and unique object names")
        if self.plate_shape not in ("circle", "rounded_square"):
            raise ValueError("Unsupported plate shape")
        if min(self.plate_base_thickness, self.plate_wall_thickness, self.plate_rim_height) <= 0:
            raise ValueError("Plate thickness and rim height must be positive")
        if self.plate_mass <= 0 or not isinstance(self.plate_movable, bool):
            raise ValueError("Plate requires positive mass and a boolean movable flag")
        if self.plate_wall_thickness >= self.plate_radius:
            raise ValueError("Plate wall must be thinner than half width")
        if (
            self.plate_shape == "rounded_square"
            and not self.plate_wall_thickness < self.plate_corner_radius < self.plate_radius
        ):
            raise ValueError("Plate corner radius must exceed rim thickness and be smaller than half width")

    def plate_distance(self, points):
        """Signed distance in metres to the outer plate boundary, for centred XY points."""
        import numpy as np

        points = np.asarray(points)
        if self.plate_shape == "circle":
            return np.linalg.norm(points, axis=-1) - self.plate_radius
        q = np.abs(points) - (self.plate_radius - self.plate_corner_radius)
        return (
            np.linalg.norm(np.maximum(q, 0), axis=-1)
            + np.minimum(np.max(q, axis=-1), 0)
            - self.plate_corner_radius
        )


@dataclass
class TaskStatus:
    success: bool = False
    failure: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)


class Task:
    """A task selects a shared scene and owns its instruction and scoring state."""

    def __init__(self, definition: dict):
        self.definition = copy.deepcopy(definition)
        self.id = definition["id"]
        self.instruction = definition["instruction"]
        self.params = self.definition.setdefault("params", {})

    def scene(self) -> SceneSpec:
        """Read geometry from the selected XML; custom tasks can override this hook."""
        from ..config import SimConfig
        from ..xml_scene import load_scene, scene_path

        path = scene_path(self.definition)
        if path is None:
            raise NotImplementedError("Task requires an XML scene or a scene() implementation")
        spec = SceneSpec([])
        load_scene(path, spec, self.params, SimConfig(images=False))
        return spec

    def reset(self, env: Any) -> None:
        pass

    def evaluate(self, env: Any) -> TaskStatus:
        raise NotImplementedError

    def make_oracle(self, env: Any) -> Any:
        raise NotImplementedError(f"Task {self.id} has no scripted baseline")


def _expand_document(document: dict, path: Path) -> list[dict]:
    """Normalize family catalogs and legacy single-task YAML into resolved definitions."""
    from ..config import merge

    if not isinstance(document, dict):
        raise ValueError(f"Invalid task definition: {path}")
    if "family" not in document or "tasks" not in document:
        return [copy.deepcopy(document)]
    family = document["family"]
    if not isinstance(family, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", family):
        raise ValueError(f"Invalid family id in {path}")
    children = document["tasks"]
    if not isinstance(children, dict) or not children:
        raise ValueError(f"Family tasks must be a nonempty mapping: {path}")
    if not {"class", "scene"} <= document.keys():
        raise ValueError(f"Family requires class and scene: {path}")
    common = document.get("params", {})
    if not isinstance(common, dict):
        raise ValueError(f"Family params must be a mapping: {path}")
    result = []
    for task_id, child in children.items():
        location = f"{path}: tasks.{task_id}"
        if not isinstance(child, dict) or set(child) - {"params", "instruction"}:
            raise ValueError(f"Subtask supports only params and instruction: {location}")
        if not isinstance(child.get("params", {}), dict):
            raise ValueError(f"Subtask params must be a mapping: {location}")
        params = merge(common, child.get("params", {}))
        instruction = child.get("instruction")
        if instruction is None:
            template = document.get("instruction_template")
            if not isinstance(template, str):
                raise ValueError(f"Missing instruction or instruction_template: {location}")
            try:
                instruction = template.format(**params)
            except (KeyError, ValueError, AttributeError, IndexError) as exc:
                raise ValueError(f"Invalid instruction template at {location}: {exc}") from exc
        result.append(
            {
                "id": task_id,
                "family": family,
                "class": document["class"],
                "scene": document["scene"],
                "instruction": instruction,
                "params": params,
            }
        )
    return result


class _UniqueKeyLoader(yaml.SafeLoader):
    """Do not silently replace a duplicate task ID within one YAML mapping."""

    def construct_mapping(self, node, deep=False):
        keys = set()
        for key_node, _ in node.value:
            # Preserve SafeLoader's standard merge/anchor support for legacy YAML.
            if key_node.tag == "tag:yaml.org,2002:merge":
                continue
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, (str, int, float, bool, type(None))):
                raise ValueError(f"Invalid YAML key at {key_node.start_mark}")
            if key in keys:
                raise ValueError(f"Duplicate YAML key {key!r} at {key_node.start_mark}")
            keys.add(key)
        return super().construct_mapping(node, deep=deep)


def discover(task_paths: list[str] | None = None) -> dict[str, dict]:
    paths = [Path(__file__).resolve().parents[1] / "task_configs", *map(Path, task_paths or [])]
    result = {}
    for directory in paths:
        if not directory.is_dir():
            raise ValueError(f"Task directory does not exist: {directory}")
        for path in sorted(directory.glob("*.yaml")):
            with path.open() as source:
                document = yaml.load(source, Loader=_UniqueKeyLoader)
            for definition in _expand_document(document, path):
                if not {"id", "class", "instruction"} <= definition.keys():
                    raise ValueError(f"Invalid task definition: {path}")
                task_id = definition["id"]
                if not isinstance(task_id, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", task_id):
                    raise ValueError(f"Invalid task id in {path}")
                if task_id in result:
                    raise ValueError(f"Duplicate task id {task_id}: {path}")
                if not isinstance(definition["instruction"], str) or not definition["instruction"].strip():
                    raise ValueError(f"Invalid instruction for {task_id}: {path}")
                if not isinstance(definition.get("params", {}), dict):
                    raise ValueError(f"Invalid params for {task_id}: {path}")
                result[task_id] = {**definition, "source": str(path)}
    return dict(sorted(result.items()))


def make_task(task_id: str, task_paths: list[str] | None = None) -> Task:
    definitions = discover(task_paths)
    if task_id not in definitions:
        raise ValueError(f"Unknown task {task_id}; available: {list(definitions)}")
    definition = definitions[task_id]
    module, name = definition["class"].split(":", 1)
    cls = getattr(importlib.import_module(module), name)
    if not issubclass(cls, Task):
        raise TypeError("Task class must implement the Task interface")
    try:
        return cls(definition)
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError(f"Invalid task {task_id} in {definition['source']}: {exc}") from exc
