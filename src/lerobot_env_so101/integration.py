"""Optional LeRobot adapters; no serial port or physical follower is opened."""

from __future__ import annotations

from dataclasses import dataclass, field

import gymnasium as gym
import numpy as np
from lerobot.configs import FeatureType, PolicyFeature
from lerobot.envs.configs import EnvConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot

from .config import JOINTS, SimConfig


@EnvConfig.register_subclass("so101_bench")
@dataclass
class SO101EnvConfig(EnvConfig):
    task: str = "place_red_in_plate"
    task_paths: list[str] = field(default_factory=list)
    width: int = 640
    height: int = 480
    episode_seconds: float = 60
    render_backend: str = "osmesa"

    def __post_init__(self) -> None:
        self.features = {
            "action": PolicyFeature(FeatureType.ACTION, (6,)),
            "agent_pos": PolicyFeature(FeatureType.STATE, (6,)),
        }
        self.features_map = {"action": "action", "agent_pos": "observation.state"}
        for name in ("front", "wrist"):
            self.features[f"pixels/{name}"] = PolicyFeature(FeatureType.VISUAL, (self.height, self.width, 3))
            self.features_map[f"pixels/{name}"] = f"observation.images.{name}"

    @property
    def gym_kwargs(self) -> dict:
        return {
            "task_id": self.task,
            "task_paths": self.task_paths,
            "cfg": SimConfig(
                control_hz=self.fps,
                width=self.width,
                height=self.height,
                episode_seconds=self.episode_seconds,
                render_backend=self.render_backend,
            ),
        }

    def create_envs(self, n_envs: int = 1, use_async_envs: bool = False) -> dict:
        from functools import partial

        from .env import SO101Env

        if n_envs < 1:
            raise ValueError("n_envs must be positive")
        factories = [partial(SO101Env, **self.gym_kwargs) for _ in range(n_envs)]
        kwargs = {"autoreset_mode": gym.vector.AutoresetMode.SAME_STEP}
        if use_async_envs and n_envs > 1:
            vec = gym.vector.AsyncVectorEnv(factories, context="spawn", **kwargs)
        else:
            vec = gym.vector.SyncVectorEnv(factories, **kwargs)
        return {"so101_bench": {0: vec}}


@RobotConfig.register_subclass("so101_sim")
@dataclass
class SO101SimRobotConfig(RobotConfig):
    task: str = "place_red_in_plate"
    task_paths: list[str] = field(default_factory=list)
    seed: int = 0
    sim: SimConfig = field(default_factory=SimConfig)


class SO101SimRobot(Robot):
    config_class = SO101SimRobotConfig
    name = "so101_sim"

    def __init__(self, config: SO101SimRobotConfig) -> None:
        # Simulation has no motor calibration file; avoid Robot's filesystem/hardware setup.
        self.config = config
        self.robot_type = self.name
        self.id = config.id
        self.env = None
        self.observation = None
        self.last_transition = None

    @property
    def observation_features(self) -> dict:
        return {
            **self.action_features,
            **{n: (self.config.sim.height, self.config.sim.width, 3) for n in ("front", "wrist")},
        }

    @property
    def action_features(self) -> dict:
        return {f"{n}.pos": float for n in JOINTS}

    @property
    def is_connected(self) -> bool:
        return self.env is not None

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        from .env import SO101Env

        if self.is_connected:
            raise RuntimeError("Simulation already connected")
        self.env = SO101Env(self.config.task, self.config.sim, self.config.task_paths)
        self.reset_episode(self.config.seed)

    def reset_episode(self, seed: int) -> dict:
        if not self.is_connected:
            raise RuntimeError("Simulation is disconnected")
        self.observation, info = self.env.reset(seed=seed)
        return info

    def get_observation(self) -> dict:
        if not self.is_connected:
            raise RuntimeError("Simulation is disconnected")
        return {
            **dict(zip(self.action_features, self.observation["agent_pos"].tolist(), strict=True)),
            **self.observation["pixels"],
        }

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.is_connected:
            raise RuntimeError("Simulation is disconnected")
        requested = np.array([action[f"{n}.pos"] for n in JOINTS], dtype=np.float32)
        self.last_transition = self.env.step(requested)
        self.observation = self.last_transition[0]
        return dict(zip(self.action_features, self.env.applied_action.tolist(), strict=True))

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        if self.env is not None:
            self.env.close()
            self.env = None
