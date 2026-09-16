from __future__ import annotations

import csv
import importlib.metadata
import json
import subprocess
import time
from collections.abc import Callable
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from .config import RunConfig
from .env import SO101Env
from .policy import ActionBuffer, PolicyWorker
from .recording import EpisodeSink, FileEpisodeSink, write_json
from .scene import ASSET_DIR
from .tasks.base import discover


def provenance() -> dict:
    versions = {}
    for name in ("lerobot_env_so101", "mujoco", "gymnasium", "numpy", "lerobot", "torch"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    repo = Path(__file__).resolve().parents[3] / "lerobot"
    git = {}
    for key, args in (("commit", ["rev-parse", "HEAD"]), ("dirty", ["status", "--porcelain"])):
        proc = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)
        git[key] = proc.stdout.strip() if proc.returncode == 0 else None
    return {
        "versions": versions,
        "lerobot_git": git,
        "robot_asset": json.loads((ASSET_DIR / "manifest.json").read_text()),
    }


def run_episode(
    cfg: RunConfig,
    task_id: str,
    seed: int,
    episode: int,
    directory: str | Path,
    worker: PolicyWorker | None = None,
    sink_factory: Callable[..., EpisodeSink] = FileEpisodeSink,
) -> dict:
    env = SO101Env(task_id, cfg.sim, cfg.task_paths)
    sink = viewer = None
    buffer = ActionBuffer(episode)
    latency, underruns, overruns = [], 0, 0
    setup_started = time.monotonic()
    info = {"is_success": False, "failure_reason": None}
    control_started = None
    error = None
    try:
        obs, reset_info = env.reset(seed=seed)
        sink = sink_factory(
            directory,
            cfg,
            {
                "config": asdict(cfg),
                "task": env.task_impl.definition,
                "seed": seed,
                "episode": episode,
                "sampled": reset_info["sampled"],
                "initial_state": env.snapshot(),
            },
        )
        Path(directory, "scene.xml").write_text(env.model_xml)
        if cfg.display:
            from .viewer import ThreeViewWindow

            viewer = ThreeViewWindow(env)
        oracle = env.task_impl.make_oracle(env) if cfg.policy.backend == "scripted" else None
        if worker:
            worker.reset(episode)
            worker.submit(episode, 0, obs, env.task_description)
            response = worker.receive(cfg.policy.rpc_timeout)
            buffer.install(response[1], response[2], response[3], 0)
            latency.append(response[4])
        control_started = time.monotonic()
        last_fresh = control_started
        setup_seconds = control_started - setup_started
        while True:
            loop_started = time.monotonic()
            step = env.step_index
            if oracle:
                requested = oracle.action()
            elif cfg.mode == "sync":
                if not buffer.actions:
                    worker.submit(episode, step, obs, env.task_description)
                    response = worker.receive(cfg.policy.rpc_timeout)
                    buffer.install(response[1], response[2], response[3], step)
                    latency.append(response[4])
                requested = buffer.take(step)
                if requested is None:
                    raise RuntimeError("Synchronous policy returned an action gap")
            else:
                response = worker.receive()
                if response:
                    buffer.install(response[1], response[2], response[3], step)
                    latency.append(response[4])
                if not worker.inflight and len(buffer.actions) <= cfg.policy.chunk_size * cfg.refill_fraction:
                    worker.submit(episode, step, obs, env.task_description)
                requested = buffer.take(step)
                if requested is None:
                    underruns += 1
                    requested = env.applied_action.copy()
                    if time.monotonic() - last_fresh >= cfg.starvation_seconds:
                        info = {**info, "failure_reason": "action_starvation"}
                        break
                else:
                    last_fresh = time.monotonic()
            before = obs
            snapshot = env.snapshot()
            frames = dict(env.frames)
            obs, reward, terminated, truncated, info = env.step(requested)
            info["observation_step"] = step
            info["wall_elapsed"] = time.monotonic() - control_started
            info["inference_inflight"] = bool(worker and worker.inflight)
            sink.append(before, requested, env.applied_action, info, snapshot, frames)
            if viewer and not viewer.draw(info, cfg.mode):
                info["failure_reason"] = "user_closed_window"
                break
            if terminated or truncated:
                break
            if cfg.mode == "realtime":
                deadline = control_started + env.step_index / cfg.sim.control_hz
                if time.monotonic() > deadline + 0.002:
                    overruns += 1
                time.sleep(max(0, deadline - time.monotonic()))
            elif cfg.display:
                time.sleep(max(0, 1 / cfg.sim.control_hz - (time.monotonic() - loop_started)))
    except (Exception, KeyboardInterrupt) as exc:
        error = f"{type(exc).__name__}: {exc}"
        info = {**info, "is_success": False, "failure_reason": "runtime_error"}
    finally:
        elapsed = time.monotonic() - (control_started or setup_started)
        sim_seconds = env.step_index / cfg.sim.control_hz
        summary = {
            "task": task_id,
            "seed": seed,
            "episode": episode,
            "mode": cfg.mode,
            "policy": cfg.policy.type if worker else "scripted",
            "backend": cfg.policy.backend,
            "success": bool(info["is_success"]),
            "failure_reason": info.get("failure_reason"),
            "error": error,
            "steps": env.step_index,
            "sim_seconds": sim_seconds,
            "wall_seconds": elapsed,
            "setup_seconds": locals().get("setup_seconds", elapsed),
            "real_time_factor": sim_seconds / max(elapsed, 1e-9),
            "underrun_steps": underruns,
            "expired_actions": buffer.expired,
            "rejected_actions": buffer.rejected,
            "control_overruns": overruns,
            "inference_seconds": latency,
            "inference_p50": float(np.median(latency)) if latency else None,
            "inference_p95": float(np.percentile(latency, 95)) if latency else None,
            "metrics": info.get("metrics", {}),
            "path": str(directory),
        }
        summary["realtime_timing_valid"] = cfg.mode != "realtime" or summary["real_time_factor"] >= 0.9
        try:
            if sink:
                sink.finish(summary)
            else:
                Path(directory).mkdir(parents=True, exist_ok=True)
                write_json(Path(directory) / "summary.json", summary)
        finally:
            try:
                if viewer:
                    viewer.close()
            finally:
                env.close()
    return summary


def run(cfg: RunConfig) -> tuple[Path, dict]:
    registry = discover(cfg.task_paths)
    tasks = list(registry) if cfg.tasks == ["all"] else cfg.tasks
    if not tasks or any(t not in registry for t in tasks):
        raise ValueError(f"Unknown tasks; available: {list(registry)}")
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
    output = Path(cfg.output) / f"{stamp}_{cfg.mode}_{cfg.policy.backend}"
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "config.json", asdict(cfg))
    write_json(output / "provenance.json", provenance())
    worker = None
    summaries = []
    try:
        if cfg.policy.backend != "scripted":
            started = time.monotonic()
            worker = PolicyWorker(cfg.policy, cfg.sim.width, cfg.sim.height)
            write_json(output / "policy_load.json", {"seconds": time.monotonic() - started})
        episode = 0
        for task in tasks:
            for seed in range(cfg.seed, cfg.seed + cfg.episodes):
                path = output / task / f"seed_{seed:06d}"
                print(f"Running {task}, seed={seed}, mode={cfg.mode}", flush=True)
                result = run_episode(cfg, task, seed, episode, path, worker)
                summaries.append(result)
                print(
                    f"  success={result['success']} reason={result['failure_reason']} "
                    f"error={result['error']}",
                    flush=True,
                )
                episode += 1
                if result["error"] or result["failure_reason"] == "user_closed_window":
                    return output, summarize(summaries, output)
    except BaseException as exc:
        write_json(output / "error.json", {"type": type(exc).__name__, "message": str(exc)})
        raise
    finally:
        if worker:
            worker.close()
    return output, summarize(summaries, output)


def summarize(episodes: list[dict], output: str | Path) -> dict:
    per_task = {}
    for task in sorted({e["task"] for e in episodes}):
        subset = [e for e in episodes if e["task"] == task]
        valid = [e for e in subset if e["realtime_timing_valid"] and not e["error"]]
        per_task[task] = {
            "episodes": len(subset),
            "successes": sum(e["success"] for e in subset),
            "success_rate": float(np.mean([e["success"] for e in subset])),
            "valid_timing_episodes": len(valid),
            "valid_timing_success_rate": float(np.mean([e["success"] for e in valid])) if valid else None,
            "runtime_errors": sum(e["error"] is not None for e in subset),
        }
    report = {
        "episodes": episodes,
        "per_task": per_task,
        "overall_success_rate": float(np.mean([e["success"] for e in episodes])) if episodes else 0.0,
    }
    write_json(Path(output) / "results.json", report)
    columns = [
        "task",
        "seed",
        "mode",
        "policy",
        "backend",
        "success",
        "failure_reason",
        "steps",
        "sim_seconds",
        "wall_seconds",
        "inference_p50",
        "inference_p95",
        "real_time_factor",
        "underrun_steps",
        "expired_actions",
        "control_overruns",
        "realtime_timing_valid",
        "error",
    ]
    with (Path(output) / "episodes.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(episodes)
    return report
