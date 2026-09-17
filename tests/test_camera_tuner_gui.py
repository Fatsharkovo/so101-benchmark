import os
import time

import numpy as np
import pytest

from lerobot_env_so101.camera_profile import load_profile
from lerobot_env_so101.camera_tuner import CameraTunerWindow, CameraTuningSession
from lerobot_env_so101.config import load_config

pytestmark = pytest.mark.skipif(os.environ.get("SO101_TEST_GUI") != "1", reason="Requires Tk/GLFW display")


def pump(root):
    for _ in range(6):
        root.update()
        time.sleep(0.015)


def make_window(path):
    import tkinter as tk

    config = load_config(
        overrides={"video": False, "sim": {"width": 128, "height": 96, "render_backend": "glfw"}}
    )
    root = tk.Tk()
    session = CameraTuningSession(config, path)
    window = CameraTunerWindow(root, session)
    return root, session, window


def test_drag_preview_save_reopen_and_overwrite(tmp_path):
    path = tmp_path / "front.yaml"
    root, session, window = make_window(path)
    try:
        pump(root)
        initial = window.frame.copy()
        model = session.env.model
        state = session.env.snapshot()
        window.sliders["fovy"].set(68)
        pump(root)
        assert session.pose.fovy == 68
        assert window.variables["fovy"].get() == "68.0"
        assert np.any(window.frame != initial)
        assert session.env.snapshot() == state and session.env.model is model
        window.variables["height_cm"].set("42.5")
        window.entries["height_cm"].event_generate("<KeyRelease>")
        # Explicit Return tests the numeric commit even when the window manager lacks focus.
        window.entry_changed("height_cm")
        pump(root)
        assert session.pose.height_cm == 42.5
        assert window.save()
        assert load_profile(path).height_cm == 42.5
        assert not session.dirty
    finally:
        window.shutdown()
    root, session, window = make_window(path)
    try:
        pump(root)
        assert session.pose.fovy == 68 and session.pose.height_cm == 42.5
        window.sliders["fovy"].set(52)
        pump(root)
        assert window.save()
        assert load_profile(path).fovy == 52
        window.restore_scene()
        assert session.pose.fovy == 45
        window.restore_saved()
        assert session.pose.fovy == 52
        content = path.read_bytes()
        window.variables["fovy"].set("nan")
        window.entry_changed("fovy")
        assert not window.save()
        assert path.read_bytes() == content and session.pose.fovy == 52
        assert window.entries["fovy"].instate(["invalid"])
    finally:
        window.shutdown()


def test_close_cancel_failed_save_discard_and_successful_save(tmp_path, monkeypatch):
    from tkinter import messagebox

    path = tmp_path / "front.yaml"
    root, session, window = make_window(path)
    try:
        pump(root)
        window.sliders["fovy"].set(60)
        monkeypatch.setattr(messagebox, "askyesnocancel", lambda *a, **kw: None)
        window.close()
        assert not window.closed and not path.exists()
        monkeypatch.setattr(messagebox, "askyesnocancel", lambda *a, **kw: True)

        def fail():
            raise OSError("disk full")

        original = session.save
        monkeypatch.setattr(session, "save", fail)
        window.close()
        assert not window.closed and "保存失败" in window.status.get()
        monkeypatch.setattr(session, "save", original)
        window.close()
        assert window.closed and load_profile(path).fovy == 60
    finally:
        window.shutdown()
    root, session, window = make_window(path)
    try:
        pump(root)
        window.sliders["fovy"].set(70)
        monkeypatch.setattr(messagebox, "askyesnocancel", lambda *a, **kw: False)
        window.close()
        assert window.closed and load_profile(path).fovy == 60
    finally:
        window.shutdown()
