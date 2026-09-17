"""Front camera pose conversion and replaceable per-user calibration preferences."""

from __future__ import annotations

import math
import os
import tempfile
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .config import SimConfig

# Keys, user-facing labels, lower/upper limits; positions are cm, angles are degrees.
CAMERA_CONTROLS = (
    ("x_cm", "前后距离 X / cm", 0.0, 200.0),
    ("y_cm", "左右平移 Y / cm", -100.0, 100.0),
    ("height_cm", "高度 Z / cm", 1.0, 200.0),
    ("pitch_deg", "下俯角 / °", -90.0, 90.0),
    ("yaw_deg", "左右转向 / °", -180.0, 180.0),
    ("roll_deg", "画面旋转 / °", -180.0, 180.0),
    ("fovy", "垂直视场角 fovy / °", 1.0, 179.0),
)


def profile_path() -> Path:
    """Resolve each time so reconfiguration and XDG preferences never use a stale cache."""
    return (
        Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
        / "so101-benchmark/front_camera.yaml"
    )


def pose_rotation(pitch_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    """Look toward -X at yaw zero; positive pitch points down, roll is about optical -Z."""
    pitch = math.radians(pitch_deg)
    base = np.array(
        [[0, -math.sin(pitch), math.cos(pitch)], [1, 0, 0], [0, math.cos(pitch), math.sin(pitch)]]
    )
    return (
        Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
        @ base
        @ Rotation.from_euler("z", -roll_deg, degrees=True).as_matrix()
    )


@dataclass(frozen=True)
class FrontCameraPose:
    x_cm: float
    y_cm: float
    height_cm: float
    pitch_deg: float
    yaw_deg: float
    roll_deg: float
    fovy: float

    def __post_init__(self) -> None:
        for key, label, low, high in CAMERA_CONTROLS:
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{label}: expected a finite number")
            if not low <= value <= high:
                raise ValueError(f"{label}: expected {low:g} to {high:g}, got {value:g}")

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x_cm, self.y_cm, self.height_cm]) / 100

    @property
    def rotation(self) -> np.ndarray:
        return pose_rotation(self.pitch_deg, self.yaw_deg, self.roll_deg)

    @property
    def quaternion(self) -> list[float]:
        x, y, z, w = Rotation.from_matrix(self.rotation).as_quat()
        return [float(w), float(x), float(y), float(z)]

    @classmethod
    def from_env(cls, env) -> FrontCameraPose:
        camera = env.data.cam("front")
        rotation = camera.xmat.reshape(3, 3)
        forward = -rotation[:, 2]
        horizontal = float(np.linalg.norm(forward[:2]))
        pitch = math.degrees(math.atan2(-forward[2], horizontal))
        yaw = math.degrees(math.atan2(-forward[1], -forward[0])) if horizontal > 1e-8 else 0
        remainder = pose_rotation(pitch, yaw, 0).T @ rotation
        roll = -math.degrees(math.atan2(remainder[1, 0], remainder[0, 0]))
        # Rounding numerical noise, not slider precision, preserves loaded poses.
        values = [*camera.xpos * 100, pitch, yaw, roll, float(env.model.cam("front").fovy[0])]
        return cls(*(round(float(v), 10) for v in values))

    def apply(self, env) -> None:
        """Change only the fixed camera. Physics state and renderer instances are preserved."""
        camera = env.model.cam("front")
        if camera.bodyid[0] != 0 or camera.mode[0] != env.mj.mjtCamLight.mjCAMLIGHT_FIXED:
            raise ValueError("Front camera tuning requires a fixed camera in worldbody")
        camera.pos[:] = self.position
        camera.quat[:] = self.quaternion
        camera.fovy[:] = self.fovy
        env.mj.mj_camlight(env.model, env.data)

    def configure(self, config: SimConfig) -> None:
        """Replace all conflicting orientation fields while retaining display preferences."""
        current = dict(config.cameras.get("front", {}))
        for key in ("position", "pos", "target", "euler", "quat", "xyaxes", "axisangle", "zaxis"):
            current.pop(key, None)
        current.update(position=self.position.tolist(), quat=self.quaternion, fovy=self.fovy, fixed=True)
        config.cameras["front"] = current


def load_profile(path: Path | None = None) -> FrontCameraPose | None:
    path = path or profile_path()
    try:
        raw = yaml.safe_load(path.read_text())
        if not isinstance(raw, dict) or set(raw) != {"front"} or not isinstance(raw["front"], dict):
            raise ValueError("Expected a YAML mapping with a front section")
        if set(raw["front"]) != {field.name for field in fields(FrontCameraPose)}:
            raise ValueError("Expected x_cm, y_cm, height_cm, pitch_deg, yaw_deg, roll_deg and fovy")
        return FrontCameraPose(**raw["front"])
    except FileNotFoundError:
        return None
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot load camera profile {path}: {exc}") from exc


def save_profile(pose: FrontCameraPose, path: Path | None = None) -> None:
    """Atomically replace the same personal file; never write shared scene XML."""
    path = path or profile_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", dir=path.parent, prefix=".front_camera_", suffix=".tmp", delete=False
        ) as stream:
            temporary = Path(stream.name)
            stream.write("# Personal front camera: positions in cm, angles in degrees.\n")
            yaml.safe_dump({"front": asdict(pose)}, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def apply_local_profile(config: SimConfig) -> Path | None:
    """Load once per teleoperation startup, before snapshots and environment creation."""
    path = profile_path()
    pose = load_profile(path)
    if pose is not None:
        pose.configure(config)
        return path
    return None


def apply_camera_overrides(root, config: SimConfig) -> None:
    """Use one precedence rule for editable XML and Python-generated task scenes."""
    from .scene import look_at, numbers

    for name, values in config.cameras.items():
        camera = root.find(f".//camera[@name='{name}']")
        if camera is None:
            raise ValueError(f"Unknown scene camera {name}")
        if "position" in values or "pos" in values:
            camera.set("pos", numbers(values.get("position", values.get("pos"))))
        if any(key in values for key in ("quat", "target", "euler")):
            for key in ("quat", "euler", "xyaxes", "axisangle", "zaxis"):
                camera.attrib.pop(key, None)
            if "quat" in values:
                camera.set("quat", numbers(values["quat"]))
            elif "target" in values:
                camera.set("xyaxes", look_at(np.fromstring(camera.get("pos"), sep=" "), values["target"]))
            else:
                camera.set("euler", numbers(values["euler"]))
        if "fovy" in values:
            camera.set("fovy", str(values["fovy"]))
