"""Read an SO-101 Leader in its existing environment and drive three simulation views."""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


def read_leader(args: argparse.Namespace) -> None:
    """Stream calibrated joint angles; never send position targets to hardware."""
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    calibration_dir = HF_LEROBOT_CALIBRATION / "teleoperators" / "so_leader"
    if not (calibration_dir / f"{args.leader_id}.json").is_file():
        raise FileNotFoundError(f"Missing existing calibration for {args.leader_id}")
    leader = SO101Leader(SO101LeaderConfig(port=args.port, id=args.leader_id, use_degrees=True))

    def stop(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        leader.bus.connect()
        if not leader.is_calibrated:
            raise RuntimeError("Leader calibration differs from saved calibration; recalibration required")
        leader.configure()
        while True:
            started = time.monotonic()
            action = leader.get_action()
            print(json.dumps({"action": action, "time": time.monotonic()}), flush=True)
            time.sleep(max(0, 1 / args.control_hz - (time.monotonic() - started)))
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        if leader.is_connected:
            leader.disconnect()


def run(args: argparse.Namespace) -> None:
    """Run three persistent views and record successful episodes with the official writer."""
    os.environ["MUJOCO_GL"] = "glfw"
    from dataclasses import asdict

    import numpy as np

    from lerobot_env_so101.camera_profile import apply_local_profile
    from lerobot_env_so101.config import JOINTS, load_config
    from lerobot_env_so101.env import SO101Env
    from lerobot_env_so101.recording import write_json
    from lerobot_env_so101.runner import provenance
    from lerobot_env_so101.teleop import DatasetWriterProcess, TeleopSession
    from lerobot_env_so101.teleop_viewer import SeparateViews

    overrides = {"display": True}
    if args.task:
        overrides["tasks"] = [args.task]
    config = load_config(args.config, overrides)
    camera_profile = apply_local_profile(config.sim)
    if camera_profile:
        print(f"Using personal front camera: {camera_profile}", flush=True)
    if len(config.tasks) != 1:
        raise ValueError("Teleoperation needs a single YAML task or --task override")
    task = config.tasks[0] if config.tasks != ["all"] else "stack_blue_on_red"
    output = args.output or Path(config.output) / f"teleop_{time.time_ns()}"
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "config.json", asdict(config))
    metadata = provenance()
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], capture_output=True, text=True, cwd=Path(__file__).resolve().parents[1]
    )
    metadata["benchmark_commit"] = revision.stdout.strip() if revision.returncode == 0 else None
    write_json(output / "provenance.json", metadata)
    env = SO101Env(task, config.sim, config.task_paths)
    viewer = process = writer = session = None
    samples: queue.Queue = queue.Queue(maxsize=1)
    started = time.monotonic()
    steps = 0
    reason = "duration_elapsed"
    cleanup_error = None
    try:
        env.reset(seed=config.seed)
        viewer = SeparateViews(env)
        writer = DatasetWriterProcess(
            config.recording.writer_python or args.leader_python,
            {
                "root": str((output / "dataset").resolve()),
                "repo_id": config.recording.repo_id or f"local/{task}",
                "fps": config.sim.control_hz,
                "width": config.sim.width,
                "height": config.sim.height,
            },
            output / "writer.log",
        )
        session = TeleopSession(env, writer, config, config.seed)
        child_env = os.environ.copy()
        child_env.pop("PYTHONPATH", None)
        process = subprocess.Popen(
            [
                args.leader_python,
                "-u",
                str(Path(__file__).resolve()),
                "--read-leader",
                "--port",
                args.port,
                "--leader-id",
                args.leader_id,
                "--control-hz",
                str(config.sim.control_hz),
            ],
            stdout=subprocess.PIPE,
            text=True,
            env=child_env,
        )

        def receive() -> None:
            for line in process.stdout:
                try:
                    sample = json.loads(line)
                    if not isinstance(sample, dict) or "action" not in sample:
                        continue
                except (ValueError, TypeError):
                    continue
                try:
                    samples.get_nowait()
                except queue.Empty:
                    pass
                samples.put_nowait(sample)

        threading.Thread(target=receive, daemon=True).start()
        latest = None
        next_report = 0.0
        deadline = time.monotonic()
        print(f"Recording output: {output.resolve()}", flush=True)
        while args.seconds is None or time.monotonic() - started < args.seconds:
            try:
                session.poll()
                if process.poll() is not None:
                    raise RuntimeError(f"Leader reader exited: {process.returncode}")
                try:
                    latest = samples.get_nowait()
                except queue.Empty:
                    pass
                now = time.monotonic()
                if not viewer.draw(session.info, session.label()):
                    reason = "window_closed"
                    break
                if viewer.reset_requested:
                    viewer.reset_requested = False
                    session.reset()
                    viewer.start_requested = False
                if viewer.start_requested:
                    viewer.start_requested = False
                    if latest is not None and now - latest["time"] <= 1:
                        session.start()
                if session.state != "ERROR":
                    if latest is None:
                        if now - started > 30:
                            raise TimeoutError("No Leader sample within 30 seconds")
                    else:
                        if now - latest["time"] > 1:
                            raise TimeoutError("Leader readings stopped for more than one second")
                        action = np.array([latest["action"][f"{joint}.pos"] for joint in JOINTS])
                        session.step(action)
                        steps += 1
                if now >= next_report:
                    print(
                        json.dumps(
                            {
                                "state": session.state,
                                "saved_episodes": session.saved_episodes,
                                "step": env.step_index,
                                "error": session.error,
                            }
                        ),
                        flush=True,
                    )
                    next_report = now + 5
            except Exception as exc:
                if session.state != "ERROR":
                    print(str(exc), file=sys.stderr, flush=True)
                    session.fail(exc)
                    reason = str(exc)
                # Keep errors visible in the existing window until the user exits.
                if not viewer.draw(session.info, session.label()):
                    break
            deadline = max(deadline + 1 / config.sim.control_hz, time.monotonic())
            time.sleep(max(0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        reason = "interrupted"
    except Exception as exc:
        reason = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            process.stdout.close()
        try:
            if writer is not None:
                writer.close()
        except Exception as exc:
            cleanup_error = str(exc)
        finally:
            if viewer is not None:
                viewer.close()
            env.close()
        summary = {
            "steps": steps,
            "resets": session.resets if session else 0,
            "saved_episodes": writer.saved_episodes if writer else 0,
            "reason": reason,
            "error": cleanup_error or (session.error if session else None),
            "seconds": time.monotonic() - started,
        }
        write_json(output / "summary.json", summary)
        print(json.dumps(summary), flush=True)
        if cleanup_error:
            raise RuntimeError(cleanup_error)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--read-leader", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--leader-python", default=sys.executable)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).resolve().parents[1] / "configs/fixed.yaml"
    )
    parser.add_argument("--task")
    parser.add_argument("--control-hz", type=int, default=30, help=argparse.SUPPRESS)
    parser.add_argument(
        "--seconds", type=float, help="Optional whole-session time limit; episode duration comes from YAML"
    )
    parser.add_argument("--output", type=Path, help="New session directory (must not exist)")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.read_leader:
        read_leader(args)
    else:
        run(args)


if __name__ == "__main__":
    main()
