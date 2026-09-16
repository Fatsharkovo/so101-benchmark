import numpy as np

from lerobot_env_so101.config import PolicyConfig, RunConfig, SimConfig
from lerobot_env_so101.runner import run_episode, summarize


class ManualClock:
    """Make action deadlines independent of software-rendering speed on the test host."""

    now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class DelayedWorker:
    """Deliberately slower than its action horizon; does not load any model."""

    def __init__(self, clock):
        self.clock = clock
        self.inflight = False
        self.requests = 0

    def reset(self, episode):
        self.episode = episode

    def submit(self, episode, step, observation, instruction):
        assert not self.inflight
        self.requests += 1
        self.inflight = True
        self.ready = self.clock.monotonic() + (0 if self.requests == 1 else 0.11)
        self.reply = ("actions", episode, step, np.tile(observation["agent_pos"], (2, 1)), 0.11)

    def receive(self, timeout=0):
        if self.inflight and self.clock.monotonic() >= self.ready:
            self.inflight = False
            return self.reply
        return None


def test_realtime_discards_late_actions_and_stops_starvation(tmp_path, monkeypatch):
    clock = ManualClock()
    monkeypatch.setattr("lerobot_env_so101.runner.time", clock)
    cfg = RunConfig(
        tasks=["stack_blue_on_red"],
        episodes=1,
        video=False,
        mode="realtime",
        starvation_seconds=0.3,
        sim=SimConfig(width=32, height=32, episode_seconds=2),
        policy=PolicyConfig(backend="local", chunk_size=2),
    )
    result = run_episode(cfg, cfg.tasks[0], 0, 41, tmp_path / "episode", DelayedWorker(clock))
    assert result["error"] is None
    assert result["failure_reason"] == "action_starvation"
    assert result["expired_actions"] >= 2
    assert result["underrun_steps"] > 0


def test_timing_invalid_results_have_separate_aggregate(tmp_path):
    episodes = [
        {"task": "a", "success": True, "realtime_timing_valid": False, "error": None},
        {"task": "a", "success": False, "realtime_timing_valid": True, "error": None},
    ]
    report = summarize(episodes, tmp_path)
    assert report["per_task"]["a"]["success_rate"] == 0.5
    assert report["per_task"]["a"]["valid_timing_success_rate"] == 0
    assert report["per_task"]["a"]["valid_timing_episodes"] == 1
