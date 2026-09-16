"""Privileged-state diagnostic controller using IK and physical finger contact."""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares


def solve_ik(env, position, initial=None):
    data = env.mj.MjData(env.model)
    data.qpos[:] = env.data.qpos
    site = env.model.site("gripperframe").id
    initial = env.data.qpos[env.qadr[:5]].copy() if initial is None else np.array(initial)
    low, high = env.limits[:5, 0] + 1e-5, env.limits[:5, 1] - 1e-5

    def residual(q):
        data.qpos[env.qadr[:5]] = q
        env.mj.mj_forward(env.model, data)
        r = data.site_xmat[site].reshape(3, 3)
        return np.r_[60 * (data.site_xpos[site] - position), 2 * (r[:, 0] - [0, 0, -1]), r[0, 1]]

    result = least_squares(
        residual,
        np.clip(initial, low, high),
        bounds=(low, high),
        max_nfev=100,
        ftol=1e-8,
        xtol=1e-8,
        gtol=1e-8,
    )
    data.qpos[env.qadr[:5]] = result.x
    env.mj.mj_forward(env.model, data)
    error = np.linalg.norm(data.site_xpos[site] - position)
    if error > 0.012:
        raise ValueError(f"IK target {np.round(position, 3)} is unreachable (error {error:.3f} m)")
    return result.x


class PickPlaceOracle:
    def __init__(self, env, target, base=None):
        self.env, self.target, self.base = env, target, base
        self.stage = 0
        self.elapsed = 0
        self.waypoints = None
        self.start_action = env.from_sim(env.data.qpos[env.qadr])

    def _plan(self):
        env = self.env
        pick = env.object_position(self.target)
        place = (
            env.object_position(self.base)
            + [0, 0, (env.object_sizes[self.base] + env.object_sizes[self.target]) / 2]
            if self.base
            else np.array([*env.sampled["plate_xy"], 0.006 + env.object_sizes[self.target] / 2])
        )
        pick = pick + [0, 0, 0.001]
        place = place + [0, 0, 0.003]
        high_pick = pick + [0, 0, 0.075]
        high_place = place + [0, 0, 0.075]
        self.place = place
        self.high_place = high_place
        motions = [
            (high_pick, 70, 1.5),
            (pick, 70, 1.2),
            (pick, 0, 0.8),
            (high_pick, 0, 1.2),
            (high_place, 0, 1.5),
            (place, 0, 1.2),
            (place, 70, 0.7),
            (high_place, 70, 1.2),
        ]
        self.waypoints = []
        initial = env.data.qpos[env.qadr[:5]].copy()
        for position, grip, seconds in motions:
            initial = solve_ik(env, position, initial)
            q = np.r_[initial, 0]
            action = env.from_sim(q)
            action[5] = grip
            self.waypoints.append((action, round(seconds * env.cfg.control_hz)))

    def action(self):
        if self.waypoints is None:
            self._plan()
        if self.stage >= len(self.waypoints):
            return self.waypoints[-1][0].copy()
        if self.stage == 5 and self.elapsed % 3 == 0:
            offset = self.env.data.site("gripperframe").xpos - self.env.object_position(self.target)
            initial = self.env.data.qpos[self.env.qadr[:5]]
            q = solve_ik(self.env, self.place + offset, initial)
            goal = self.env.from_sim(np.r_[q, 0])
            goal[5] = 0
            self.waypoints[5] = (goal, self.waypoints[5][1])
            release = goal.copy()
            release[5] = 70
            self.waypoints[6] = (release, self.waypoints[6][1])
        goal, steps = self.waypoints[self.stage]
        self.elapsed += 1
        t = min(1.0, self.elapsed / steps)
        blend = t * t * (3 - 2 * t)
        action = self.start_action + blend * (goal - self.start_action)
        if self.elapsed >= steps:
            self.start_action = goal.copy()
            self.stage += 1
            self.elapsed = 0
            if self.stage == 4:
                offset = self.env.data.site("gripperframe").xpos - self.env.object_position(self.target)
                offset[2] = 0
                initial = self.env.data.qpos[self.env.qadr[:5]].copy()
                for index, position, grip in (
                    (4, self.high_place, 0),
                    (5, self.place, 0),
                    (6, self.place, 70),
                    (7, self.high_place, 70),
                ):
                    initial = solve_ik(self.env, position + offset, initial)
                    adjusted = self.env.from_sim(np.r_[initial, 0])
                    adjusted[5] = grip
                    self.waypoints[index] = (adjusted, self.waypoints[index][1])
        return action
