"""Private local IPC worker for the official LeRobot v3 writer interpreter."""

from __future__ import annotations

import sys
import traceback
from multiprocessing.connection import Connection
from pathlib import Path


def serve(connection: Connection) -> None:
    dataset = None
    try:
        from lerobot.configs import RGBEncoderConfig
        from lerobot.datasets.dataset_metadata import CODEBASE_VERSION
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        from lerobot_env_so101.recording import dataset_features, write_json

        if CODEBASE_VERSION != "v3.0":
            raise RuntimeError(f"Expected LeRobot v3.0 dataset writer, found {CODEBASE_VERSION}")
        config = connection.recv()
        dataset = LeRobotDataset.create(
            repo_id=config["repo_id"],
            root=Path(config["root"]),
            fps=config["fps"],
            features=dataset_features(config["width"], config["height"]),
            robot_type="so101_sim",
            streaming_encoding=True,
            metadata_buffer_size=1,
            video_backend="pyav",
            rgb_encoder=RGBEncoderConfig(vcodec="h264", preset="ultrafast", crf=20),
        )
        connection.send({"event": "ready"})
        while True:
            command, payload = connection.recv()
            if command == "frame":
                dataset.add_frame(payload)
                connection.send({"event": "frame"})
            elif command == "save":
                count = dataset.meta.total_episodes
                directory = Path(config["root"]).parent / "episodes" / f"episode_{count:06d}"
                directory.mkdir(parents=True, exist_ok=False)
                (directory / "scene.xml").write_text(payload.pop("scene_xml"))
                write_json(directory / "metadata.json", payload)
                dataset.save_episode()
                connection.send({"event": "saved", "count": dataset.meta.total_episodes})
            elif command == "discard":
                dataset.clear_episode_buffer()
                connection.send({"event": "discarded"})
            elif command == "close":
                dataset.clear_episode_buffer()
                dataset.finalize()
                connection.send({"event": "closed", "count": dataset.meta.total_episodes})
                break
            else:
                raise ValueError(f"Unknown writer command: {command}")
    except (EOFError, BrokenPipeError):
        pass
    except BaseException:
        try:
            connection.send({"event": "error", "message": traceback.format_exc()})
        except (OSError, EOFError):
            pass
    finally:
        if dataset is not None:
            try:
                dataset.clear_episode_buffer()
            finally:
                dataset.finalize()
        connection.close()


if __name__ == "__main__":
    serve(Connection(int(sys.argv[1])))
