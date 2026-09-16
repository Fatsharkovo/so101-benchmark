"""Persistent overview, front and wrist windows for teleoperation."""

from __future__ import annotations

import glfw
import mujoco as mj
import numpy as np

from .env import SO101Env
from .viewer import ThreeViewWindow


class SeparateViews(ThreeViewWindow):
    def __init__(self, env: SO101Env) -> None:
        super().__init__(env)
        self.reset_requested = False
        self.start_requested = False
        self.bound_model = env.model
        self.extra = []
        glfw.set_window_title(self.window, "SO-101 Overview | Space: start recording | R: reset | Esc: stop")
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
        if key == glfw.KEY_SPACE and action == glfw.PRESS:
            self.start_requested = True

    def draw(self, info: dict, mode: str) -> bool:
        self.bind_model()
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
            label = mode
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
            glfw.window_should_close(window) for window in [self.window, *(item[1] for item in self.extra)]
        )

    def bind_model(self) -> None:
        """Replace model-dependent resources while retaining all native windows and the orbit camera."""
        if self.bound_model is self.env.model:
            return
        glfw.make_context_current(self.window)
        self.context.free()
        self.context = mj.MjrContext(self.env.model, mj.mjtFontScale.mjFONTSCALE_100)
        self.scene = mj.MjvScene(self.env.model, maxgeom=5000)
        for index, (name, window, context) in enumerate(self.extra):
            glfw.make_context_current(window)
            context.free()
            self.extra[index] = (name, window, mj.MjrContext(self.env.model, mj.mjtFontScale.mjFONTSCALE_100))
        self.bound_model = self.env.model

    def close(self) -> None:
        for _, window, context in self.extra:
            glfw.make_context_current(window)
            if context is not None:
                context.free()
            glfw.destroy_window(window)
        super().close()
