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
            time.sleep(max(0, 1 / 30 - (time.monotonic() - started)))
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        if leader.is_connected:
            leader.disconnect()


def run(args: argparse.Namespace) -> None:
    """Run the simulator in the uv environment with a separate Leader reader process."""
    os.environ["MUJOCO_GL"] = "glfw"
    import glfw
    import mujoco as mj
    import numpy as np
    from PIL import Image

    from lerobot_env_so101.config import JOINTS, SimConfig
    from lerobot_env_so101.env import SO101Env
    from lerobot_env_so101.viewer import ThreeViewWindow

    class SeparateViews(ThreeViewWindow):
        def __init__(self, env: SO101Env) -> None:
            super().__init__(env)
            self.reset_requested = False
            self.extra = []
            glfw.set_window_title(self.window, "SO-101 Overview | R: reset | Esc: stop")
            glfw.set_window_size(self.window, 800, 660)
            glfw.set_window_pos(self.window, 20, 60)
            try:
                for index, name in enumerate(("front", "wrist")):
                    window = glfw.create_window(480, 360, f"SO-101 {name}", None, None)
                    if not window:
                        raise RuntimeError(f"Cannot open {name} window")
                    self.extra.append((name, window, None))
                    glfw.set_window_pos(window, 840, 40 + index * 390)
                    glfw.set_key_callback(window, self._key)
                    glfw.make_context_current(window)
                    glfw.swap_interval(0)
                    context = mj.MjrContext(env.model, mj.mjtFontScale.mjFONTSCALE_100)
                    self.extra[-1] = (name, window, context)
            except BaseException:
                self.close()
                raise

        def _key(self, window, key, scancode, action, mods) -> None:
            super()._key(window, key, scancode, action, mods)
            if key == glfw.KEY_R and action == glfw.PRESS:
                self.reset_requested = True

        def draw(self, info: dict, mode: str) -> bool:
            glfw.make_context_current(self.window)
            width, height = glfw.get_framebuffer_size(self.window)
            if width and height:
                rect = mj.MjrRect(0, 0, width, height)
                mj.mjv_updateScene(
                    self.env.model,
                    self.env.data,
                    self.option,
                    None,
                    self.camera,
                    mj.mjtCatBit.mjCAT_ALL,
                    self.scene,
                )
                mj.mjr_render(rect, self.scene, self.context)
                label = f"{self.env.task} | {mode} | step {self.env.step_index} | R: reset | Esc: stop"
                mj.mjr_overlay(
                    mj.mjtFont.mjFONT_NORMAL,
                    mj.mjtGridPos.mjGRID_TOPLEFT,
                    rect,
                    label,
                    "",
                    self.context,
                )
                glfw.swap_buffers(self.window)
            for name, window, context in self.extra:
                glfw.make_context_current(window)
                width, height = glfw.get_framebuffer_size(window)
                if not width or not height:
                    continue
                frame = self.env.frames[name]
                ys = np.arange(height) * frame.shape[0] // height
                xs = np.arange(width) * frame.shape[1] // width
                pixels = np.ascontiguousarray(frame[ys[:, None], xs][::-1])
                mj.mjr_drawPixels(pixels.reshape(-1), None, mj.MjrRect(0, 0, width, height), context)
                glfw.swap_buffers(window)
            glfw.poll_events()
            return not any(
                glfw.window_should_close(window)
                for window in [self.window, *(item[1] for item in self.extra)]
            )

        def close(self) -> None:
            for _, window, context in self.extra:
                glfw.make_context_current(window)
                if context is not None:
                    context.free()
                glfw.destroy_window(window)
            super().close()

    cfg = SimConfig(width=640, height=480, render_backend="glfw", episode_seconds=args.seconds)
    cfg.randomization["layout"]["enabled"] = False
    env = SO101Env(args.task, cfg)
    viewer = process = None
    samples: queue.Queue = queue.Queue(maxsize=1)
    started = time.monotonic()
    steps = resets = 0
    minimum = maximum = None
    info = {}
    reason = "duration_elapsed"
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        env.reset(seed=0)
        viewer = SeparateViews(env)
        viewer.draw({}, "Waiting for Leader")
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
            ],
            stdout=subprocess.PIPE,
            text=True,
            env=child_env,
        )

        def receive() -> None:
            for line in process.stdout:
                try:
                    sample = json.loads(line)
                    if "action" not in sample:
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
        while time.monotonic() - started < args.seconds:
            if process.poll() is not None:
                raise RuntimeError(f"Leader reader exited: {process.returncode}")
            try:
                latest = samples.get_nowait()
            except queue.Empty:
                pass
            now = time.monotonic()
            if latest is None:
                if now - started > 30:
                    raise TimeoutError("No Leader sample within 30 seconds")
                mode = "Waiting for Leader"
            else:
                if now - latest["time"] > 1:
                    raise TimeoutError("Leader readings stopped for more than one second")
                action = np.array([latest["action"][f"{joint}.pos"] for joint in JOINTS])
                minimum = action.copy() if minimum is None else np.minimum(minimum, action)
                maximum = action.copy() if maximum is None else np.maximum(maximum, action)
                if not env.done:
                    _, _, _, _, info = env.step(action)
                    steps += 1
                mode = "LIVE" if not env.done else "Episode finished; R to reset"
                if now >= next_report:
                    print(
                        json.dumps(
                            {
                                "status": mode,
                                "steps": steps,
                                "leader_degrees": action.round(2).tolist(),
                                "success": info.get("is_success", False),
                            }
                        ),
                        flush=True,
                    )
                    for name, frame in env.frames.items():
                        Image.fromarray(frame).save(args.output / f"{name}.png")
                    next_report = now + 5
            if not viewer.draw(info, mode):
                reason = "window_closed"
                break
            if viewer.reset_requested:
                viewer.close()
                viewer = None
                env.reset(seed=0)
                viewer = SeparateViews(env)
                info = {}
                resets += 1
            deadline = max(deadline + 1 / cfg.control_hz, time.monotonic())
            time.sleep(max(0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        reason = "interrupted"
    except BaseException as exc:
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
        if viewer is not None:
            viewer.close()
        env.close()
        summary = {
            "steps": steps,
            "resets": resets,
            "reason": reason,
            "seconds": time.monotonic() - started,
            "leader_range": None if minimum is None else dict(zip(JOINTS, (maximum - minimum).tolist())),
            "success": info.get("is_success", False),
        }
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--read-leader", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--leader-python", default=sys.executable)
    parser.add_argument("--task", default="stack_blue_on_red")
    parser.add_argument("--seconds", type=float, default=300)
    parser.add_argument("--output", type=Path, default=Path("outputs") / f"teleop_{time.time_ns()}")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.read_leader:
        read_leader(args)
    else:
        run(args)


if __name__ == "__main__":
    main()
