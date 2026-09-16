from __future__ import annotations

import json
from pathlib import Path
from typing import Protocol

import numpy as np

from .config import RunConfig


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value)}")


def write_json(path: str | Path, value: object) -> None:
    Path(path).write_text(json.dumps(value, indent=2, default=json_default, allow_nan=False) + "\n")


class EpisodeSink(Protocol):
    def append(
        self,
        observation: dict,
        requested: np.ndarray,
        applied: np.ndarray,
        info: dict,
        snapshot: dict,
        frames: dict,
    ) -> None: ...
    def finish(self, summary: dict) -> None: ...
    def close(self) -> None: ...


class VideoWriter:
    def __init__(self, path: str | Path, width: int, height: int, fps: int) -> None:
        import av

        self.av = av
        self.container = av.open(str(path), mode="w")
        self.stream = self.container.add_stream("libx264", rate=fps)
        self.stream.width, self.stream.height = width, height
        self.stream.pix_fmt = "yuv420p"
        self.stream.options = {"crf": "20", "preset": "ultrafast"}
        self.closed = False

    def append(self, image: np.ndarray) -> None:
        frame = self.av.VideoFrame.from_ndarray(image, format="rgb24")
        for packet in self.stream.encode(frame):
            self.container.mux(packet)

    def close(self) -> None:
        if not self.closed:
            for packet in self.stream.encode():
                self.container.mux(packet)
            self.container.close()
            self.closed = True


class FileEpisodeSink:
    def __init__(self, path: str | Path, config: RunConfig, metadata: dict) -> None:
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        write_json(self.path / "metadata.json", metadata)
        self.log = (self.path / "transitions.jsonl").open("w")
        self.videos = (
            {
                n: VideoWriter(
                    self.path / f"{n}.mp4", config.sim.width, config.sim.height, config.sim.control_hz
                )
                for n in ("overview", "front", "wrist")
            }
            if config.video
            else {}
        )

    def append(
        self,
        observation: dict,
        requested: np.ndarray,
        applied: np.ndarray,
        info: dict,
        snapshot: dict,
        frames: dict,
    ) -> None:
        record = {
            "observation_state": observation["agent_pos"],
            "requested_action": requested,
            "applied_action": applied,
            "info": info,
            "snapshot": snapshot,
        }
        self.log.write(json.dumps(record, default=json_default, allow_nan=False) + "\n")
        for name, writer in self.videos.items():
            writer.append(frames[name])

    def finish(self, summary: dict) -> None:
        self.close()
        write_json(self.path / "summary.json", summary)

    def close(self) -> None:
        self.log.close()
        for writer in self.videos.values():
            writer.close()


def dataset_features(width: int = 640, height: int = 480) -> dict:
    """LeRobotDataset feature contract; action records the executed target."""
    from lerobot.utils.feature_utils import hw_to_dataset_features

    from .config import JOINTS

    motors = {f"{n}.pos": float for n in JOINTS}
    return {
        **hw_to_dataset_features(
            {**motors, "front": (height, width, 3), "wrist": (height, width, 3)},
            "observation",
            use_video=True,
        ),
        **hw_to_dataset_features(motors, "action", use_video=False),
    }


def dataset_frame(observation: dict, applied_action: np.ndarray, instruction: str) -> dict:
    return {
        "observation.state": np.array(observation["agent_pos"], np.float32),
        "action": np.array(applied_action, np.float32),
        "task": instruction,
        **{f"observation.images.{n}": image for n, image in observation["pixels"].items()},
    }
