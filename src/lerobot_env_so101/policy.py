from __future__ import annotations

import multiprocessing as mp
import time
import traceback
from dataclasses import dataclass

import numpy as np

from .config import JOINTS, PolicyConfig


def validate_chunk(values, maximum):
    array = np.asarray(values, dtype=np.float32)
    if (
        array.ndim != 2
        or array.shape[1] != 6
        or not 1 <= len(array) <= maximum
        or not np.isfinite(array).all()
    ):
        raise ValueError(f"Invalid action chunk: shape={array.shape}; expected finite (K,6), K <= {maximum}")
    return array


def camera_name(name):
    return name.removeprefix("observation.images.")


class LocalPolicy:
    def __init__(self, cfg: PolicyConfig, width, height):
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        self.cfg = cfg
        if not cfg.checkpoint:
            raise ValueError("policy.checkpoint is required")
        config = PreTrainedConfig.from_pretrained(cfg.checkpoint)
        if config.type != cfg.type:
            raise ValueError(f"Checkpoint type {config.type} != configured {cfg.type}")
        if config.input_features["observation.state"].shape != (6,) or config.output_features[
            "action"
        ].shape != (6,):
            raise ValueError("Checkpoint must use six SO-101 state and action components")
        names = getattr(config, "action_feature_names", None)
        if names is not None and list(names) != [f"{n}.pos" for n in JOINTS]:
            raise ValueError("Checkpoint joint ordering does not match SO-101")
        if config.n_obs_steps != 1:
            raise ValueError(
                "This adapter supports the standard single-observation pi0/pi05/SmolVLA configurations"
            )
        config.device = cfg.device
        self.policy = get_policy_class(cfg.type).from_pretrained(cfg.checkpoint, config=config).to(cfg.device)
        self.policy.eval()
        self.pre, self.post = make_pre_post_processors(
            config,
            pretrained_path=cfg.checkpoint,
            preprocessor_overrides={
                "device_processor": {"device": cfg.device},
                "rename_observations_processor": {"rename_map": {}},
            },
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )
        self.torch = torch
        self.chunk_size = min(cfg.chunk_size, config.n_action_steps, config.chunk_size)
        available = {
            cfg.rename_map.get(f"observation.images.{n}", f"observation.images.{n}")
            for n in ("front", "wrist")
        }
        required = {k for k in config.image_features if "empty_camera" not in k}
        if not required <= available:
            raise ValueError(f"Missing checkpoint cameras: {required - available}; configure rename_map")

    def reset(self):
        self.policy.reset()
        self.pre.reset()
        self.post.reset()

    def predict(self, observation, instruction, step):
        torch = self.torch
        batch = {
            "observation.state": torch.tensor(observation["agent_pos"]).unsqueeze(0),
            "task": [instruction],
        }
        for name, image in observation["pixels"].items():
            key = self.cfg.rename_map.get(f"observation.images.{name}", f"observation.images.{name}")
            batch[key] = torch.from_numpy(image.copy()).permute(2, 0, 1).unsqueeze(0).float() / 255
        with torch.inference_mode():
            batch = self.pre(batch)
            chunk = self.policy.predict_action_chunk(batch)[:, : self.chunk_size]
            actions = [self.post(chunk[:, i]).detach().cpu().numpy()[0] for i in range(chunk.shape[1])]
        return validate_chunk(actions, self.cfg.chunk_size)

    def close(self):
        pass


class RemotePolicy:
    def __init__(self, cfg, width, height):
        import grpc
        from lerobot.async_inference.helpers import RemotePolicyConfig
        from lerobot.transport import services_pb2_grpc
        from lerobot.transport.utils import grpc_channel_options
        from lerobot.utils.feature_utils import hw_to_dataset_features

        if not cfg.server or not cfg.checkpoint:
            raise ValueError(
                "Remote inference requires explicit server address and server-side checkpoint path"
            )
        self.cfg = cfg
        self.channel = grpc.insecure_channel(cfg.server, options=grpc_channel_options(initial_backoff="0.1s"))
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        features = {f"{n}.pos": float for n in JOINTS}
        features.update(
            {
                camera_name(cfg.rename_map.get(f"observation.images.{n}", n)): (height, width, 3)
                for n in ("front", "wrist")
            }
        )
        self.setup = RemotePolicyConfig(
            cfg.type,
            cfg.checkpoint,
            hw_to_dataset_features(features, "observation", False),
            cfg.chunk_size,
            cfg.device,
        )

    def reset(self):
        import pickle

        from lerobot.transport import services_pb2 as pb

        self.stub.Ready(pb.Empty(), timeout=self.cfg.setup_timeout)
        self.stub.SendPolicyInstructions(
            pb.PolicySetup(data=pickle.dumps(self.setup)), timeout=self.cfg.setup_timeout
        )

    def predict(self, observation, instruction, step):
        import pickle

        from lerobot.async_inference.helpers import TimedObservation
        from lerobot.transport import services_pb2 as pb
        from lerobot.transport.utils import send_bytes_in_chunks

        raw = dict(zip((f"{n}.pos" for n in JOINTS), observation["agent_pos"].tolist(), strict=True))
        raw.update(
            {
                camera_name(self.cfg.rename_map.get(f"observation.images.{n}", n)): image
                for n, image in observation["pixels"].items()
            }
        )
        raw["task"] = instruction
        obs = TimedObservation(time.time(), step, raw, must_go=True)
        self.stub.SendObservations(
            send_bytes_in_chunks(pickle.dumps(obs), pb.Observation, silent=True), timeout=self.cfg.rpc_timeout
        )
        reply = self.stub.GetActions(pb.Empty(), timeout=self.cfg.rpc_timeout)
        if not reply.data:
            raise RuntimeError("PolicyServer returned no actions")
        actions = pickle.loads(reply.data)  # Trusted, explicitly configured LeRobot server protocol.
        if [a.get_timestep() for a in actions] != list(range(step, step + len(actions))):
            raise ValueError("PolicyServer returned mismatched action timesteps")
        if len(actions) > 1 and not np.allclose(
            np.diff([a.get_timestamp() for a in actions]), 1 / self.cfg.server_fps, atol=1e-5
        ):
            raise ValueError("PolicyServer action timestamps do not match the configured control frequency")
        return validate_chunk([a.get_action().detach().cpu().numpy() for a in actions], self.cfg.chunk_size)

    def close(self):
        self.channel.close()


def _worker(connection, cfg, width, height):
    backend = None
    try:
        backend = (LocalPolicy if cfg.backend == "local" else RemotePolicy)(cfg, width, height)
        connection.send(("ready",))
        while True:
            message = connection.recv()
            if message[0] == "close":
                break
            if message[0] == "reset":
                backend.reset()
                connection.send(("reset", message[1]))
            else:
                _, episode, step, obs, instruction = message
                started = time.monotonic()
                actions = backend.predict(obs, instruction, step)
                connection.send(("actions", episode, step, actions, time.monotonic() - started))
    except EOFError:
        pass
    except BaseException:
        connection.send(("error", traceback.format_exc()))
    finally:
        if backend is not None:
            backend.close()
        connection.close()


class PolicyWorker:
    """One model process and at most one request in flight; physics stays in the parent."""

    def __init__(self, cfg, width, height):
        self.cfg = cfg
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_worker, args=(child, cfg, width, height), daemon=True)
        self.process.start()
        child.close()
        self.inflight = False
        try:
            self.receive(cfg.setup_timeout)
        except BaseException:
            self.close()
            raise

    def receive(self, timeout=0):
        if not self.connection.poll(timeout):
            if timeout:
                raise TimeoutError("Policy worker timed out")
            if not self.process.is_alive():
                raise RuntimeError("Policy worker exited unexpectedly")
            return None
        message = self.connection.recv()
        if message[0] == "error":
            raise RuntimeError(message[1])
        if message[0] == "actions":
            self.inflight = False
        return message

    def reset(self, episode):
        if self.inflight:
            self.receive(self.cfg.rpc_timeout)
        self.connection.send(("reset", episode))
        reply = self.receive(self.cfg.setup_timeout)
        if reply != ("reset", episode):
            raise RuntimeError("Unexpected policy reset response")

    def submit(self, episode, step, observation, instruction):
        if self.inflight:
            raise RuntimeError("Only one inference request may be in flight")
        self.connection.send(("predict", episode, step, observation, instruction))
        self.inflight = True

    def close(self):
        if self.process.is_alive():
            try:
                self.connection.send(("close",))
            except (BrokenPipeError, OSError):
                pass
            self.process.join(timeout=1)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=5)
        self.connection.close()


@dataclass
class ActionBuffer:
    episode: int
    expired: int = 0
    rejected: int = 0

    def __post_init__(self):
        self.actions = {}

    def install(self, episode, first_step, actions, current_step):
        if episode != self.episode:
            self.rejected += len(actions)
            return
        for offset, action in enumerate(actions):
            step = first_step + offset
            if step < current_step:
                self.expired += 1
            else:
                self.actions[step] = action.copy()

    def take(self, step):
        for stale in [k for k in self.actions if k < step]:
            del self.actions[stale]
            self.expired += 1
        return self.actions.pop(step, None)
