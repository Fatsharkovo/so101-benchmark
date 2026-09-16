from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from .config import load_config
from .tasks.base import discover


def main():
    parser = argparse.ArgumentParser(description="SO-101 MuJoCo benchmark")
    parser.add_argument("command", choices=["list", "preview", "eval", "replay"])
    parser.add_argument("--config")
    parser.add_argument("--task", help="Comma-separated task ids, or all")
    parser.add_argument("--episodes", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--mode", choices=["sync", "realtime"])
    parser.add_argument("--output")
    parser.add_argument("--display", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--video", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--episode-dir", help="Episode directory to replay")
    args = parser.parse_args()
    overrides = {
        key: getattr(args, key)
        for key in ("episodes", "seed", "mode", "output", "display", "video")
        if getattr(args, key) is not None
    }
    if args.task:
        overrides["tasks"] = args.task.split(",")
    cfg = load_config(args.config, overrides)
    os.environ["MUJOCO_GL"] = cfg.sim.render_backend
    if args.command == "list":
        for task_id, definition in discover(cfg.task_paths).items():
            print(f"{task_id}: {definition['instruction']}")
    elif args.command == "eval":
        from .runner import run

        path, report = run(cfg)
        print(json.dumps({"output": str(path), "per_task": report["per_task"]}, indent=2))
        if any(e["error"] for e in report["episodes"]):
            raise SystemExit(1)
    elif args.command == "preview":
        from .env import SO101Env
        from .viewer import ThreeViewWindow

        task = next(iter(discover(cfg.task_paths))) if cfg.tasks == ["all"] else cfg.tasks[0]
        env = SO101Env(task, cfg.sim, cfg.task_paths)
        viewer = None
        try:
            env.reset(seed=cfg.seed)
            if cfg.display:
                import time

                viewer = ThreeViewWindow(env)
                while viewer.draw({}, "preview"):
                    time.sleep(1 / 30)
            else:
                import av

                output = Path(cfg.output) / "preview"
                output.mkdir(parents=True, exist_ok=True)
                for name, image in env.frames.items():
                    av.VideoFrame.from_ndarray(image, format="rgb24").to_image().save(output / f"{name}.png")
                print(output.resolve())
        finally:
            if viewer:
                viewer.close()
            env.close()
    else:
        if not args.episode_dir:
            parser.error("replay requires --episode-dir")
        replay(args.episode_dir, cfg)


def replay(directory, cfg):
    from .config import SimConfig
    from .env import SO101Env
    from .recording import VideoWriter

    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    sim = SimConfig(**metadata["config"]["sim"])
    sim.images = True
    sim.render_backend = cfg.sim.render_backend
    env = SO101Env(metadata["task"]["id"], sim, metadata["config"]["task_paths"])
    viewer, writers = None, {}
    try:
        env.reset(seed=metadata["seed"])
        if cfg.display:
            from .viewer import ThreeViewWindow

            viewer = ThreeViewWindow(env)
        if cfg.video:
            output = directory / "replay"
            output.mkdir(exist_ok=True)
            writers = {
                n: VideoWriter(output / f"{n}.mp4", sim.width, sim.height, sim.control_hz)
                for n in ("overview", "front", "wrist")
            }
        with (directory / "transitions.jsonl").open() as file:
            for line in file:
                row = json.loads(line)
                state = row["snapshot"]
                env.data.qpos[:] = state["qpos"]
                env.data.qvel[:] = state["qvel"]
                env.data.ctrl[:] = state["ctrl"]
                env.data.time = state["time"]
                env.mj.mj_forward(env.model, env.data)
                env.observe()
                env.step_index = row["info"]["observation_step"]
                for name, writer in writers.items():
                    writer.append(env.frames[name])
                if viewer and not viewer.draw(row["info"], "replay"):
                    break
    finally:
        for writer in writers.values():
            writer.close()
        if viewer:
            viewer.close()
        env.close()


if __name__ == "__main__":
    main()
