from __future__ import annotations

import copy
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
COLORS = {
    "red": (0.85, 0.05, 0.04),
    "blue": (0.04, 0.15, 0.9),
    "green": (0.05, 0.65, 0.1),
    "yellow": (0.95, 0.8, 0.02),
    "orange": (1.0, 0.32, 0.02),
}


@dataclass
class SimConfig:
    control_hz: int = 30
    physics_hz: int = 600
    interpolate_actions: bool = False
    episode_seconds: float = 60.0
    width: int = 640
    height: int = 480
    render_backend: str = "osmesa"
    images: bool = True
    home_degrees: list[float] = field(default_factory=lambda: [0, -70, 70, 60, 0, 0])
    joint_signs: list[float] = field(default_factory=lambda: [1] * 5)
    # q_sim = radians((leader_degrees - offset) * sign); wrist zero faces forward.
    joint_offsets_deg: list[float] = field(default_factory=lambda: [0, 0, 0, 0, 90])
    cameras: dict[str, Any] = field(default_factory=dict)
    randomization: dict[str, dict[str, Any]] = field(
        default_factory=lambda: {
            "layout": {"enabled": True, "position_jitter": 0.005, "yaw_deg": [-5, 5]},
            "appearance": {"enabled": False, "brightness": [0.8, 1.2], "table_gray": [0.35, 0.65]},
            "camera": {"enabled": False, "position_jitter": 0.005, "rotation_deg": 2.0},
            "size": {"enabled": False, "scale": [0.9, 1.1]},
            "physics": {"enabled": False, "mass_scale": [0.8, 1.2], "friction_scale": [0.8, 1.2]},
        }
    )

    def __post_init__(self) -> None:
        if not isinstance(self.interpolate_actions, bool):
            raise ValueError("interpolate_actions must be a boolean")
        if self.control_hz <= 0 or self.physics_hz % self.control_hz or self.physics_hz < self.control_hz:
            raise ValueError("physics_hz must be a positive integer multiple of control_hz")
        if (
            self.episode_seconds <= 0
            or min(self.width, self.height) < 16
            or self.width % 2
            or self.height % 2
        ):
            raise ValueError("Invalid episode duration or image size")
        if self.render_backend not in ("osmesa", "egl", "glfw"):
            raise ValueError("render_backend must be osmesa, egl or glfw")
        if len(self.home_degrees) != 6 or len(self.joint_signs) != 5 or len(self.joint_offsets_deg) != 5:
            raise ValueError("Expected 6 home positions and 5 body joint conversion values")
        if any(s not in (-1, 1) for s in self.joint_signs):
            raise ValueError("joint_signs must contain only -1 or 1")
        allowed = set(SimConfig.__dataclass_fields__["randomization"].default_factory())
        if set(self.randomization) - allowed:
            raise ValueError("Unknown randomization group")
        if self.randomization.get("layout", {}).get("mode", "jitter") not in ("jitter", "workspace"):
            raise ValueError("layout.mode must be jitter or workspace")
        for group in self.randomization.values():
            for key, value in group.items():
                if isinstance(value, list) and (len(value) != 2 or value[0] > value[1]):
                    raise ValueError(f"Invalid randomization range {key}: {value}")
                if key in ("scale", "mass_scale", "friction_scale", "brightness") and value[0] <= 0:
                    raise ValueError(f"{key} must be positive")
                if key in ("position_jitter", "rotation_deg") and value < 0:
                    raise ValueError(f"{key} must be nonnegative")


@dataclass
class PolicyConfig:
    backend: str = "scripted"
    type: str = "pi05"
    checkpoint: str = ""
    device: str = "cuda"
    server: str = ""
    server_fps: int = 30
    chunk_size: int = 50
    rename_map: dict[str, str] = field(default_factory=dict)
    rpc_timeout: float = 120.0
    setup_timeout: float = 600.0

    def __post_init__(self) -> None:
        if self.backend not in ("scripted", "local", "remote"):
            raise ValueError("Unknown policy backend")
        if self.type not in ("pi0", "pi05", "smolvla"):
            raise ValueError("Supported policies: pi0, pi05, smolvla")
        if min(self.chunk_size, self.rpc_timeout, self.setup_timeout, self.server_fps) <= 0:
            raise ValueError("Chunk size and timeouts must be positive")


@dataclass
class RecordingConfig:
    repo_id: str = ""
    writer_python: str = ""


@dataclass
class RunConfig:
    tasks: list[str] = field(default_factory=lambda: ["all"])
    task_paths: list[str] = field(default_factory=list)
    mode: str = "sync"
    episodes: int = 10
    seed: int = 0
    display: bool = False
    video: bool = True
    output: str = "outputs"
    starvation_seconds: float = 10.0
    refill_fraction: float = 0.5
    sim: SimConfig = field(default_factory=SimConfig)
    policy: PolicyConfig = field(default_factory=PolicyConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)

    def __post_init__(self) -> None:
        if self.mode not in ("sync", "realtime") or self.episodes < 1 or self.seed < 0:
            raise ValueError("Invalid mode, episodes or seed")
        if not 0 < self.refill_fraction <= 1 or self.starvation_seconds <= 0:
            raise ValueError("Invalid queue configuration")
        if (self.video or self.display or self.policy.backend != "scripted") and not self.sim.images:
            raise ValueError("Images are required for video, display and learned policies")
        if self.display:
            self.sim.render_backend = "glfw"
        if self.policy.backend == "remote" and self.policy.server_fps != self.sim.control_hz:
            raise ValueError("PolicyServer fps must match sim.control_hz; set policy.server_fps accordingly")


def merge(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in override.items():
        result[key] = (
            merge(result[key], value)
            if isinstance(value, dict) and isinstance(result.get(key), dict)
            else value
        )
    return result


def construct(cls: type, values: dict) -> Any:
    unknown = set(values) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"Unknown {cls.__name__} fields: {sorted(unknown)}")
    return cls(**values)


def load_config(path: str | Path | None = None, overrides: dict | None = None) -> RunConfig:
    raw = asdict(RunConfig())
    if path:
        document = yaml.safe_load(Path(path).read_text()) or {}
        raw = merge(raw, document)
    raw = merge(raw, overrides or {})
    raw["sim"] = construct(SimConfig, raw["sim"])
    raw["policy"] = construct(PolicyConfig, raw["policy"])
    raw["recording"] = construct(RecordingConfig, raw["recording"])
    config = construct(RunConfig, raw)
    if path:
        parent = Path(path).resolve().parent
        config.task_paths = [str((parent / p).resolve()) for p in config.task_paths]
    return config
