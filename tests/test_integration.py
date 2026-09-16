import json

import av
import numpy as np
import pytest

from lerobot_env_so101.config import JOINTS, RunConfig, SimConfig
from lerobot_env_so101.recording import dataset_features, dataset_frame
from lerobot_env_so101.runner import run_episode


def test_lerobot_plugin_and_vector_final_info():
    pytest.importorskip("lerobot")
    from lerobot.envs.factory import make_env, make_env_config
    from lerobot.envs.utils import preprocess_observation
    from lerobot.utils.import_utils import register_third_party_plugins

    register_third_party_plugins()
    config = make_env_config("so101_bench", width=32, height=32, episode_seconds=1 / 30)
    env = make_env(config)["so101_bench"][0]
    try:
        obs, _ = env.reset(seed=0)
        inputs = preprocess_observation(obs)
        assert inputs["observation.state"].shape == (1, 6)
        assert inputs["observation.images.front"].shape == (1, 3, 32, 32)
        _, _, done, truncated, info = env.step(np.zeros((1, 6), np.float32))
        assert truncated[0]
        assert "is_success" in info["final_info"]
    finally:
        env.close()


def test_sim_follower_accepts_leader_style_actions_and_dataset_contract():
    pytest.importorskip("lerobot")
    from lerobot_env_so101.integration import SO101SimRobot, SO101SimRobotConfig

    robot = SO101SimRobot(SO101SimRobotConfig(sim=SimConfig(images=False, width=32, height=32)))
    try:
        robot.connect()
        obs = robot.get_observation()
        assert set(robot.observation_features) == set(obs)
        applied = robot.send_action(
            {
                f"{name}.pos": float(value)
                for name, value in zip(JOINTS, [10, -60, 60, 60, 0, 50], strict=True)
            }
        )
        assert list(applied) == [f"{n}.pos" for n in JOINTS]
        features = dataset_features(32, 32)
        frame = dataset_frame(robot.observation, robot.env.applied_action, "Pick up red.")
        assert set(features) <= set(frame)
        assert features["action"]["names"] == [f"{n}.pos" for n in JOINTS]
        assert frame["action"].shape == (6,)
    finally:
        robot.disconnect()


def test_video_alignment_and_replay(tmp_path):
    from lerobot_env_so101.cli import replay

    cfg = RunConfig(
        tasks=["place_red_in_plate"],
        episodes=1,
        video=True,
        sim=SimConfig(width=64, height=48, episode_seconds=0.1),
    )
    result = run_episode(cfg, cfg.tasks[0], 0, 0, tmp_path / "episode")
    assert result["error"] is None
    rows = [json.loads(line) for line in (tmp_path / "episode/transitions.jsonl").read_text().splitlines()]
    assert len(rows) == 3
    assert [row["info"]["observation_step"] for row in rows] == [0, 1, 2]
    for name in ("overview", "front", "wrist"):
        with av.open(str(tmp_path / f"episode/{name}.mp4")) as video:
            frames = list(video.decode(video=0))
            assert len(frames) == 3
            assert frames[0].width == 64
    replay(tmp_path / "episode", cfg)
    with av.open(str(tmp_path / "episode/replay/front.mp4")) as video:
        assert len(list(video.decode(video=0))) == 3
