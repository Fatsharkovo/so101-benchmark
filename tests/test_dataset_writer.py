"""Optional real LeRobot writer checks, using its separate installed interpreter."""

import json
import os
import subprocess
import time

import numpy as np
import pytest

from lerobot_env_so101.recording import dataset_frame
from lerobot_env_so101.teleop import DatasetWriterProcess


@pytest.mark.skipif(
    not os.environ.get("SO101_DATASET_PYTHON"), reason="Set SO101_DATASET_PYTHON to LeRobot Python"
)
def test_v3_save_discard_finalize_and_reopen(tmp_path):
    python = os.environ["SO101_DATASET_PYTHON"]
    root = tmp_path / "dataset"
    writer = DatasetWriterProcess(
        python,
        {"root": str(root), "repo_id": "local/test", "fps": 30, "width": 32, "height": 32},
        tmp_path / "writer.log",
    )

    def wait_for(kind):
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            for event in writer.poll():
                assert event["event"] != "error", event
                if event["event"] == kind:
                    return event
            time.sleep(0.05)
        pytest.fail(f"No {kind} from writer")

    def frames(value):
        for index in range(3):
            observation = {
                "agent_pos": np.full(6, value + index, np.float32),
                "pixels": {n: np.full((32, 32, 3), value + index, np.uint8) for n in ("front", "wrist")},
            }
            writer.submit("frame", dataset_frame(observation, np.full(6, index, np.float32), "Test"))

    try:
        wait_for("ready")
        frames(5)
        writer.submit("discard")
        wait_for("discarded")
        frames(60)
        writer.submit("save", {"scene_xml": "<mujoco/>", "seed": 3})
        assert wait_for("saved")["count"] == 1
        frames(120)  # Unfinished exit must discard these frames too.
    finally:
        writer.close()
    assert writer.saved_episodes == 1
    info = json.loads((root / "meta/info.json").read_text())
    assert info["codebase_version"] == "v3.0"
    assert info["total_episodes"] == 1 and info["total_frames"] == 3
    check = """
import sys
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
d = LeRobotDataset("local/test", root=sys.argv[1], video_backend="pyav")
assert len(d) == 3
for i in range(3):
    row = d[i]
    np.testing.assert_allclose(row["observation.state"].numpy(), np.full(6, 60 + i))
    np.testing.assert_allclose(row["action"].numpy(), np.full(6, i))
    assert abs(float(row["timestamp"]) - i / 30) < 1e-6
    for camera in ("front", "wrist"):
        pixels = row[f"observation.images.{camera}"]
        assert tuple(pixels.shape) == (3, 32, 32)
        assert abs(float(pixels.mean()) * 255 - (60 + i)) < 4
"""
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    subprocess.run([python, "-c", check, str(root)], check=True, env=environment, timeout=120)
