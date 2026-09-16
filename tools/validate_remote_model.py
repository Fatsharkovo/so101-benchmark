"""Run real-weight evaluation against a private, temporary loopback PolicyServer."""

from __future__ import annotations

import argparse
import os
from concurrent.futures import ThreadPoolExecutor

import grpc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--type", choices=["pi0", "pi05", "smolvla"], default="pi05")
    parser.add_argument("--mode", choices=["sync", "realtime"], default="realtime")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--episodes", type=int, default=2)
    args = parser.parse_args()
    os.environ["MUJOCO_GL"] = "egl"
    from lerobot.async_inference.configs import PolicyServerConfig
    from lerobot.async_inference.policy_server import PolicyServer
    from lerobot.transport.services_pb2_grpc import add_AsyncInferenceServicer_to_server

    from lerobot_env_so101.config import PolicyConfig, RunConfig, SimConfig
    from lerobot_env_so101.runner import run

    server = grpc.server(ThreadPoolExecutor(max_workers=4))
    service = PolicyServer(PolicyServerConfig(host="127.0.0.1"))
    add_AsyncInferenceServicer_to_server(service, server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        cfg = RunConfig(
            tasks=["stack_blue_on_red"],
            episodes=args.episodes,
            mode=args.mode,
            output="outputs/remote_model_validation",
            sim=SimConfig(width=320, height=240, episode_seconds=args.seconds, render_backend="egl"),
            policy=PolicyConfig(
                backend="remote", type=args.type, checkpoint=args.checkpoint, server=f"127.0.0.1:{port}"
            ),
        )
        path, report = run(cfg)
        print(path)
        if any(e["error"] for e in report["episodes"]):
            raise SystemExit(1)
    finally:
        server.stop(5).wait()


if __name__ == "__main__":
    main()
