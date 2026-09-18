"""Seeded placement in the SO101's tabletop grasping workspace."""

from __future__ import annotations

import numpy as np

from .tasks.base import SceneSpec


def sample_workspace(
    spec: SceneSpec, sizes: dict[str, float], rng: np.random.Generator
) -> tuple[dict[str, np.ndarray], np.ndarray | None]:
    """Reassign every object's XY independently of its XML anchor.

    This conservative region is checked against the bundled robot's downward
    grasp and approach IK in tests. Exclude the near-base area where the shoulder
    housing can collide at table height. The plate is placed first because it has
    the largest footprint; shuffle cubes to avoid a fixed color priority.
    """

    def point() -> np.ndarray | None:
        xy = rng.uniform([0.12, -0.18], [0.27, 0.18])
        return xy if 0.19 <= np.linalg.norm(xy) <= 0.29 else None

    for _ in range(100):
        positions = {}
        plate_xy = None
        if spec.plate_xy is not None:
            for _ in range(200):
                plate_xy = point()
                if plate_xy is not None:
                    break
            else:
                continue
        for index in rng.permutation(len(spec.objects)):
            obj = spec.objects[index]
            radius = sizes[obj.name] / np.sqrt(2)
            for _ in range(200):
                xy = point()
                if xy is None:
                    continue
                # Circumscribed footprints remain separated at any cube yaw,
                # with another 25 mm of room for approaching the grasp.
                if any(
                    np.linalg.norm(xy - p[:2]) < radius + sizes[name] / np.sqrt(2) + 0.025
                    for name, p in positions.items()
                ):
                    continue
                if plate_xy is not None and spec.plate_distance(xy - plate_xy) < radius + 0.025:
                    continue
                positions[obj.name] = np.r_[xy, obj.position[2] + (sizes[obj.name] - obj.size) / 2]
                break
            else:
                break
        else:
            return positions, plate_xy
    raise ValueError(
        "Cannot fit all objects with grasp clearance in the SO101 workspace after 100 layouts; "
        "reduce object count or sizes"
    )
