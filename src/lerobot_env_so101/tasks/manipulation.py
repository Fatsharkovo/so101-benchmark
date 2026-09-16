from __future__ import annotations

import math

import numpy as np

from ..config import COLORS
from .base import ObjectSpec, SceneSpec, Task, TaskStatus


class ManipulationTask(Task):
    def __init__(self, definition):
        super().__init__(definition)
        for key, default in (("stable_seconds", 1), ("linear_speed", 0.01), ("angular_speed", 0.1)):
            if self.params.get(key, default) <= 0:
                raise ValueError(f"Task threshold {key} must be positive")

    def reset(self, env) -> None:
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
    def scene(self) -> SceneSpec:
        positions = [(0.14, -0.10), (0.20, -0.105), (0.235, -0.05), (0.15, -0.035), (0.21, 0.015)]
        positions = dict(zip(COLORS, positions, strict=True))
        positions.update(self.params.get("positions", {}))
        names = self.params.get("colors", list(COLORS))
        if self.params["target"] not in names or set(names) - COLORS.keys() or len(names) != len(set(names)):
            raise ValueError("Task colors must be unique supported colors and include the target")
        objects = [
            ObjectSpec(
                name,
                COLORS[name],
                (*positions[name], 0.014),
                self.params.get("cube_size", 0.025),
                self.params.get("cube_mass", 0.010),
            )
            for name in names
        ]
        self.stable_objects = [self.params["target"]]
        return SceneSpec(
            objects,
            tuple(self.params.get("plate_xy", [0.14, 0.13])),
            self.params.get("plate_radius", 0.050),
            plate_shape=self.params.get("plate_shape", "rounded_square"),
            plate_corner_radius=self.params.get("plate_corner_radius", 0.012),
            plate_base_thickness=self.params.get("plate_base_thickness", 0.002),
            plate_wall_thickness=self.params.get("plate_wall_thickness", 0.002),
            plate_rim_height=self.params.get("plate_rim_height", 0.006),
        )

    def evaluate(self, env) -> TaskStatus:
        target = self.params["target"]
        inside = env.in_plate(target, fully=True) and env.contact_bodies(target, "plate")
        wrong = [n for n in env.object_sizes if n != target and env.in_plate(n, fully=False)]
        return self.finish(
            env, inside and not wrong, {"target_in_plate": bool(inside), "wrong_in_plate": wrong}
        )


class StackBlueOnRed(ManipulationTask):
    def scene(self) -> SceneSpec:
        self.stable_objects = [self.params["target"], self.params["base"]]
        positions = {"blue": (0.19, -0.045), "red": (0.19, 0.045)}
        positions.update(self.params.get("positions", {}))
        return SceneSpec(
            [
                ObjectSpec(
                    name,
                    COLORS[name],
                    (*positions[name], 0.014),
                    self.params.get("cube_size", 0.025),
                    self.params.get("cube_mass", 0.010),
                )
                for name in ("blue", "red")
            ]
        )

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
