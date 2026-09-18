"""Teleoperation state transitions and bounded local dataset writer transport."""

from __future__ import annotations

import os
import queue
import subprocess
import threading
from dataclasses import asdict
from multiprocessing import Pipe
from pathlib import Path

from .recording import dataset_frame


class DatasetWriterProcess:
    """Keep LeRobot dependencies out of the simulator and never drop recording frames."""

    def __init__(self, python: str, config: dict, log_path: Path, queue_size: int = 60):
        parent, child = Pipe(duplex=True)
        self.connection = parent
        self.commands = queue.Queue(maxsize=queue_size)
        self.events = queue.Queue()
        self.error = None
        self.closed = False
        self.saved_episodes = 0
        self.log = log_path.open("w")
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
        self.process = subprocess.Popen(
            [python, "-u", "-m", "lerobot_env_so101.dataset_worker", str(child.fileno())],
            pass_fds=(child.fileno(),),
            env=environment,
            stdout=self.log,
            stderr=self.log,
        )
        child.close()
        self.connection.send(config)
        self.thread = threading.Thread(target=self._transfer, daemon=True)
        self.thread.start()

    def _receive(self) -> dict:
        if not self.connection.poll(120):
            raise TimeoutError("Dataset writer did not respond within 120 seconds")
        event = self.connection.recv()
        if event["event"] == "error":
            raise RuntimeError(event["message"])
        if event["event"] in ("saved", "closed"):
            self.saved_episodes = event["count"]
        return event

    def _transfer(self):
        try:
            self.events.put(self._receive())
            while True:
                command = self.commands.get()
                self.connection.send(command)
                event = self._receive()
                if event["event"] != "frame":
                    self.events.put(event)
                if command[0] == "close":
                    break
        except BaseException as exc:
            self.error = f"Dataset writer failed: {exc}. See {self.log.name}"
            self.events.put({"event": "error", "message": self.error})

    def submit(self, command: str, payload=None):
        if self.error:
            raise RuntimeError(self.error)
        try:
            self.commands.put_nowait((command, payload))
        except queue.Full as exc:
            raise RuntimeError(
                "Dataset writer queue is full; recording stopped without dropping frames"
            ) from exc

    def poll(self) -> list[dict]:
        result = []
        while True:
            try:
                result.append(self.events.get_nowait())
            except queue.Empty:
                return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.thread.is_alive() and not self.error:
                self.commands.put(("close", None), timeout=10)
                self.thread.join(timeout=120)
            if self.thread.is_alive():
                raise TimeoutError("Dataset writer did not finalize")
            if self.error:
                raise RuntimeError(self.error)
        finally:
            if self.process.poll() is None:
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
            self.connection.close()
            self.log.close()


class TeleopSession:
    """Count only acknowledged saves; keep trial motion outside scored episodes."""

    def __init__(self, env, writer, config, seed: int):
        self.env, self.writer, self.config, self.seed = env, writer, config, seed
        self.state = "STARTING"
        self.saved_episodes = 0
        self.resets = 0
        self.error = None
        self.info = {}
        self.observation = env.observe()
        self.initial_snapshot = env.snapshot()

    def fail(self, error):
        self.error = str(error)
        self.state = "ERROR"

    def _next_layout(self):
        """Advance once per reset, after any pending dataset operation is acknowledged."""
        seed = self.seed + 1
        self.observation, self.info = self.env.reset(seed=seed)
        self.seed = seed
        self.initial_snapshot = self.env.snapshot()
        self.state = "WAITING"
        self.resets += 1

    def poll(self):
        for event in self.writer.poll():
            kind = event["event"]
            if kind == "error":
                self.fail(event["message"])
            elif self.state == "ERROR":
                continue
            elif kind == "ready":
                self.state = "WAITING"
            elif kind == "saved" and self.state == "SAVING":
                self.saved_episodes = event["count"]
                self._next_layout()
            elif kind == "discarded" and self.state == "DISCARDING":
                self._next_layout()

    def start(self):
        if self.state == "WAITING":
            self.observation = self.env.restore_initial()
            self.info = {}
            self.initial_snapshot = self.env.snapshot()
            self.state = "RECORDING"

    def reset(self):
        if self.state == "RECORDING":
            self.writer.submit("discard")
            self.state = "DISCARDING"
        elif self.state == "WAITING":
            self._next_layout()

    def step(self, action):
        if self.state == "WAITING":
            self.observation, *_ = self.env.step(action, trial=True)
        elif self.state == "RECORDING":
            before = self.observation
            self.observation, _, terminated, truncated, self.info = self.env.step(action)
            self.writer.submit(
                "frame", dataset_frame(before, self.info["applied_action"], self.env.task_description)
            )
            if terminated or truncated:
                if self.info["is_success"]:
                    self.writer.submit(
                        "save",
                        {
                            "config": asdict(self.config),
                            "task": self.env.task_impl.definition,
                            "seed": self.seed,
                            "sampled": self.env.sampled,
                            "initial_state": self.initial_snapshot,
                            "result": self.info,
                            "scene_xml": self.env.model_xml,
                        },
                    )
                    self.state = "SAVING"
                else:
                    self.writer.submit("discard")
                    self.state = "DISCARDING"

    def label(self) -> str:
        status = self.state if self.error is None else f"ERROR: {self.error.splitlines()[0][:90]}"
        return (
            f"{self.env.task} | {status} | step {self.env.step_index} | "
            f"Saved episodes: {self.saved_episodes} | Space: start recording | R: reset | Esc: stop"
        )
