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
    mass: float = 0.025

    def __post_init__(self):
        if self.size <= 0 or self.mass <= 0 or len(self.position) != 3:
            raise ValueError("Objects require positive size/mass and a three-dimensional position")


@dataclass
class SceneSpec:
    objects: list[ObjectSpec]
    plate_xy: tuple[float, float] | None = None
    plate_radius: float = 0.060
    # Additional MJCF worldbody elements for custom tasks.
    worldbody_xml: str = ""
    assets_xml: str = ""

    def __post_init__(self):
        if self.plate_radius <= 0 or len({o.name for o in self.objects}) != len(self.objects):
            raise ValueError("Scene requires a positive plate radius and unique object names")


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
