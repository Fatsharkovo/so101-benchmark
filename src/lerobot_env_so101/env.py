from __future__ import annotations

import os

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .config import JOINTS, SimConfig
from .scene import make_xml
from .tasks.base import make_task


class SO101Env(gym.Env):
    metadata = {"render_modes": ["rgb_array", "human"], "render_fps": 30}

    def __init__(
        self,
        task_id: str = "place_red_in_plate",
        cfg: SimConfig | None = None,
        task_paths: list[str] | None = None,
        render_mode: str = "rgb_array",
    ) -> None:
        self.cfg = cfg or SimConfig()
        self.task_impl = make_task(task_id, task_paths)
        self.task = task_id
        self.task_description = self.task_impl.instruction
        self.render_mode = render_mode
        self._max_episode_steps = round(self.cfg.episode_seconds * self.cfg.control_hz)
        self.observation_space = spaces.Dict(
            {
                "agent_pos": spaces.Box(-360, 360, (6,), np.float32),
                "pixels": spaces.Dict(
                    {
                        n: spaces.Box(0, 255, (self.cfg.height, self.cfg.width, 3), np.uint8)
                        for n in ("front", "wrist")
                    }
                ),
            }
        )
        offsets = np.asarray(self.cfg.joint_offsets_deg, dtype=np.float32)
        self.action_space = spaces.Box(
            np.r_[offsets - 180, np.float32(0)], np.r_[offsets + 180, np.float32(100)]
        )
        self.model = self.data = self.renderer = None
        self.frames = {}
        self.step_index = 0

    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple[dict, dict]:
        super().reset(seed=seed)
        if seed is None:
            seed = int(self.np_random.integers(0, 2**31))
        os.environ.setdefault("MUJOCO_GL", self.cfg.render_backend)
        import mujoco

        self.mj = mujoco
        self.close()
        self.scene_spec = self.task_impl.scene()
        self.model_xml, self.sampled = make_xml(self.scene_spec, self.cfg, seed)
        self.model = mujoco.MjModel.from_xml_string(self.model_xml)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = np.array([self.model.joint(n).id for n in JOINTS])
        self.qadr = self.model.jnt_qposadr[self.joint_ids]
        self.dadr = self.model.jnt_dofadr[self.joint_ids]
        self.actuator_ids = np.array([self.model.actuator(n).id for n in JOINTS])
        self.limits = self.model.jnt_range[self.joint_ids].copy()
        self.object_sizes = {n: v["size"] for n, v in self.sampled["objects"].items()}
        self.robot_bodies = set()
        base = self.model.body("base").id
        for body in range(self.model.nbody):
            cursor = body
            while cursor:
                if cursor == base:
                    self.robot_bodies.add(body)
                    break
                cursor = self.model.body_parentid[cursor]
        self.applied_action = np.array(self.cfg.home_degrees, dtype=np.float32)
        q = self.to_sim(self.applied_action)
        self.data.qpos[self.qadr] = q
        self.data.ctrl[self.actuator_ids] = q
        mujoco.mj_forward(self.model, self.data)
        for _ in range(self.cfg.physics_hz // 2):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0
        self.step_index = 0
        self.done = False
        self.task_impl.reset(self)
        return self.observe(), {"is_success": False, "sampled": self.sampled, "task": self.task}

    def to_sim(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=float)
        if action.shape != (6,) or not np.isfinite(action).all():
            raise ValueError("Expected six finite joint targets")
        q = np.empty(6)
        q[:5] = np.deg2rad((action[:5] - self.cfg.joint_offsets_deg) * self.cfg.joint_signs)
        q[5] = self.limits[5, 0] + np.clip(action[5], 0, 100) / 100 * np.ptp(self.limits[5])
        return np.clip(q, self.limits[:, 0], self.limits[:, 1])

    def from_sim(self, q: np.ndarray) -> np.ndarray:
        a = np.empty(6, np.float32)
        a[:5] = np.rad2deg(q[:5]) * self.cfg.joint_signs + self.cfg.joint_offsets_deg
        a[5] = np.clip((q[5] - self.limits[5, 0]) / np.ptp(self.limits[5]) * 100, 0, 100)
        return a

    def step(self, action: np.ndarray) -> tuple[dict, float, bool, bool, dict]:
        if self.data is None or self.done:
            raise RuntimeError("Call reset before stepping a new episode")
        q = self.to_sim(action)
        self.applied_action = self.from_sim(q)
        self.data.ctrl[self.actuator_ids] = q
        for _ in range(self.cfg.physics_hz // self.cfg.control_hz):
            self.mj.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("Non-finite simulation state")
        self.step_index += 1
        status = self.task_impl.evaluate(self)
        terminated = bool(status.success or status.failure)
        truncated = self.step_index >= self._max_episode_steps and not terminated
        self.done = terminated or truncated
        info = {
            "is_success": bool(status.success),
            "failure_reason": status.failure or ("timeout" if truncated else None),
            "metrics": status.metrics,
            "applied_action": self.applied_action.copy(),
            "sim_time": float(self.data.time),
            "step": self.step_index,
        }
        return self.observe(), float(status.success), terminated, truncated, info

    def observe(self) -> dict:
        if self.cfg.images:
            if self.renderer is None:
                self.renderer = self.mj.Renderer(self.model, height=self.cfg.height, width=self.cfg.width)
            for name in ("front", "wrist", "overview"):
                self.renderer.update_scene(self.data, camera=name)
                self.frames[name] = self.renderer.render().copy()
        else:
            self.frames = {
                n: np.zeros((self.cfg.height, self.cfg.width, 3), np.uint8)
                for n in ("front", "wrist", "overview")
            }
        return {
            "agent_pos": self.from_sim(self.data.qpos[self.qadr]),
            "pixels": {n: self.frames[n] for n in ("front", "wrist")},
        }

    def render(self) -> np.ndarray | None:
        return self.frames.get("overview")

    def object_position(self, name: str) -> np.ndarray:
        return self.data.body(name).xpos.copy()

    def object_corners(self, name: str) -> np.ndarray:
        from itertools import product

        body = self.data.body(name)
        corners = np.array(list(product([-0.5, 0.5], repeat=3))) * self.object_sizes[name]
        return corners @ body.xmat.reshape(3, 3).T + body.xpos

    def in_plate(self, name: str, fully: bool = True) -> bool:
        if self.scene_spec.plate_xy is None:
            return False
        corners = self.object_corners(name)
        center = np.array(self.sampled["plate_xy"])
        if fully:
            return bool(
                np.max(self.scene_spec.plate_distance(corners[:, :2] - center))
                < -self.scene_spec.plate_wall_thickness
                and np.min(corners[:, 2]) < self.scene_spec.plate_base_thickness + 0.006
            )
        return bool(
            self.scene_spec.plate_distance(self.object_position(name)[:2] - center)
            < self.object_sizes[name] * 0.71
            and np.min(corners[:, 2])
            < self.scene_spec.plate_base_thickness + self.scene_spec.plate_rim_height + 0.006
        )

    def contact_bodies(self, a: str, b: str) -> bool:
        ids = {self.model.body(a).id, self.model.body(b).id}
        return any(
            {int(self.model.geom_bodyid[c.geom1]), int(self.model.geom_bodyid[c.geom2])} == ids
            for c in self.data.contact[: self.data.ncon]
            if c.dist <= 0.001
        )

    def touching_robot(self, name: str) -> bool:
        object_id = self.model.body(name).id
        for c in self.data.contact[: self.data.ncon]:
            a, b = int(self.model.geom_bodyid[c.geom1]), int(self.model.geom_bodyid[c.geom2])
            if c.dist <= 0.001 and (
                (a == object_id and b in self.robot_bodies) or (b == object_id and a in self.robot_bodies)
            ):
                return True
        return False

    def is_still(self, name: str, linear: float = 0.01, angular: float = 0.1) -> bool:
        velocity = np.empty(6)
        self.mj.mj_objectVelocity(
            self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, self.model.body(name).id, velocity, 0
        )
        return np.linalg.norm(velocity[:3]) < angular and np.linalg.norm(velocity[3:]) < linear

    def upright(self, name: str) -> bool:
        return self.data.body(name).xmat.reshape(3, 3)[2, 2] > np.cos(np.deg2rad(15))

    def snapshot(self) -> dict:
        return {
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "ctrl": self.data.ctrl.tolist(),
            "time": float(self.data.time),
            "objects": {n: self.object_position(n).tolist() for n in self.object_sizes},
        }

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
