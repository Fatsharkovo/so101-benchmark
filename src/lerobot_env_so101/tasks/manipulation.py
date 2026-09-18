from __future__ import annotations

import math

import numpy as np

from ..config import COLORS
from .base import Task, TaskStatus


class ManipulationTask(Task):
    def __init__(self, definition):
        super().__init__(definition)
        if type(self).scene is Task.scene:
            self.definition.setdefault("scene", self.default_scene)
        for role in self.object_roles:
            if self.params.get(role) not in COLORS:
                raise ValueError(f"{role} must name a supported color")
        if len({self.params[role] for role in self.object_roles}) != len(self.object_roles):
            raise ValueError("Target and base must be different objects")
        if "colors" in self.params:
            colors = self.params["colors"]
            if (
                not isinstance(colors, list)
                or any(c not in COLORS for c in colors)
                or len(colors) != len(set(colors))
                or any(self.params[r] not in colors for r in self.object_roles)
            ):
                raise ValueError("Task colors must be unique supported colors and include task objects")
        for key, default in (("stable_seconds", 1), ("linear_speed", 0.01), ("angular_speed", 0.1)):
            if self.params.get(key, default) <= 0:
                raise ValueError(f"Task threshold {key} must be positive")

    def reset(self, env) -> None:
        required = {self.params[role] for role in self.object_roles}
        if missing := required - env.object_sizes.keys():
            raise ValueError(f"Task {self.id}: missing scene objects {sorted(missing)}")
        if "plate" in self.stable_scene_objects and env.scene_spec.plate_xy is None:
            raise ValueError(f"Task {self.id}: scene requires a plate")
        self.stable_objects = [self.params[role] for role in self.object_roles] + list(
            self.stable_scene_objects
        )
        self.stable_steps = 0
        self.lifted = False
        self.grasped = False

    def finish(self, env, valid: bool, metrics: dict) -> TaskStatus:
        target = self.params["target"]
        touched = env.touching_robot(target)
        self.grasped |= touched
        self.lifted |= self.grasped and env.object_position(target)[2] > env.object_sizes[target] / 2 + 0.012
        valid = valid and self.lifted and not any(env.touching_robot(n) for n in env.object_sizes)
        valid &= all(
            env.is_still(n, self.params.get("linear_speed", 0.01), self.params.get("angular_speed", 0.1))
            for n in self.stable_objects
        )
        self.stable_steps = self.stable_steps + 1 if valid else 0
        failure = "object_fell" if any(env.object_position(n)[2] < -0.04 for n in env.object_sizes) else None
        return TaskStatus(
            self.stable_steps >= math.ceil(self.params.get("stable_seconds", 1) * env.cfg.control_hz),
            failure,
            {
                **metrics,
                "lifted": bool(self.lifted),
                "stable_seconds": self.stable_steps / env.cfg.control_hz,
            },
        )

    def make_oracle(self, env):
        from ..oracle import PickPlaceOracle

        return PickPlaceOracle(env, self.params["target"], self.params.get("base"))


class PlaceInPlate(ManipulationTask):
    default_scene = "place_in_plate.xml"
    object_roles = ("target",)
    stable_scene_objects = ("plate",)

    def __init__(self, definition):
        super().__init__(definition)
        tolerance = self.params.get("plate_edge_tolerance", 0.0)
        if not np.isfinite(tolerance) or not 0 <= tolerance <= 0.005:
            raise ValueError("plate_edge_tolerance must be between 0 and 0.005 metres")

    def evaluate(self, env) -> TaskStatus:
        target = self.params["target"]
        if env.data.body("plate").xpos[2] < -0.04:
            return TaskStatus(failure="plate_fell")
        inside = env.in_plate(
            target, fully=True, edge_tolerance=self.params.get("plate_edge_tolerance", 0.0)
        ) and env.contact_bodies(target, "plate")
        wrong = [n for n in env.object_sizes if n != target and env.in_plate(n, fully=False)]
        return self.finish(
            env,
            inside and not wrong,
            {"target_in_plate": bool(inside), "wrong_in_plate": wrong},
        )


class StackCubes(ManipulationTask):
    default_scene = "stack_cubes.xml"
    object_roles = ("target", "base")
    stable_scene_objects = ()

    def evaluate(self, env) -> TaskStatus:
        target, base = self.params["target"], self.params["base"]
        top, bottom = env.object_position(target), env.object_position(base)
        expected = (env.object_sizes[target] + env.object_sizes[base]) / 2
        aligned = (
            np.linalg.norm((top - bottom)[:2]) < min(env.object_sizes[target], env.object_sizes[base]) * 0.30
        )
        valid = (
            aligned
            and abs(top[2] - bottom[2] - expected) < 0.004
            and env.contact_bodies(target, base)
            and env.contact_bodies(base, "table")
            and env.upright(target)
            and env.upright(base)
        )
        return self.finish(
            env, valid, {"aligned": bool(aligned), "height_error": float(top[2] - bottom[2] - expected)}
        )


# Compatibility for external single-task YAML and saved task definitions.
StackBlueOnRed = StackCubes
