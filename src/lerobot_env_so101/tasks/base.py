from __future__ import annotations

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
    plate_rim_height: float = 0.006

    def __post_init__(self):
        if self.plate_radius <= 0 or len({o.name for o in self.objects}) != len(self.objects):
            raise ValueError("Scene requires a positive plate radius and unique object names")
        if self.plate_shape not in ("circle", "rounded_square"):
            raise ValueError("Unsupported plate shape")
        if min(self.plate_base_thickness, self.plate_wall_thickness, self.plate_rim_height) <= 0:
            raise ValueError("Plate thickness and rim height must be positive")
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
    """A task owns its scene, instruction and scoring, never a model or renderer."""

    def __init__(self, definition: dict):
        self.definition = definition
        self.id = definition["id"]
        self.instruction = definition["instruction"]
        self.params = definition.get("params", {})

    def scene(self) -> SceneSpec:
        raise NotImplementedError

    def reset(self, env: Any) -> None:
        pass

    def evaluate(self, env: Any) -> TaskStatus:
        raise NotImplementedError

    def make_oracle(self, env: Any) -> Any:
        raise NotImplementedError(f"Task {self.id} has no scripted baseline")


def discover(task_paths: list[str] | None = None) -> dict[str, dict]:
    paths = [Path(__file__).resolve().parents[1] / "task_configs", *map(Path, task_paths or [])]
    result = {}
    for directory in paths:
        if not directory.is_dir():
            raise ValueError(f"Task directory does not exist: {directory}")
        for path in sorted(directory.glob("*.yaml")):
            definition = yaml.safe_load(path.read_text())
            if not isinstance(definition, dict) or not {"id", "class", "instruction"} <= definition.keys():
                raise ValueError(f"Invalid task definition: {path}")
            task_id = definition["id"]
            if not isinstance(task_id, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", task_id):
                raise ValueError(f"Invalid task id in {path}")
            if task_id in result:
                raise ValueError(f"Duplicate task id {task_id}")
            definition = {**definition, "source": str(path)}
            result[task_id] = definition
    return result


def make_task(task_id: str, task_paths: list[str] | None = None) -> Task:
    definitions = discover(task_paths)
    if task_id not in definitions:
        raise ValueError(f"Unknown task {task_id}; available: {list(definitions)}")
    definition = definitions[task_id]
    module, name = definition["class"].split(":", 1)
    cls = getattr(importlib.import_module(module), name)
    if not issubclass(cls, Task):
        raise TypeError("Task class must implement the Task interface")
    return cls(definition)
