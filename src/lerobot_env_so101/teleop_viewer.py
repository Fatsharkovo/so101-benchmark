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
        self.show_front_camera = env.cfg.cameras.get("front", {}).get("show_pose", True)
        if self.show_front_camera:
            # Include both the base and the elevated external camera on first opening.
            self.camera.lookat[:] = (env.data.cam("front").xpos + env.data.body("base").xpos) / 2
            self.camera.distance = max(1.05, np.linalg.norm(env.data.cam("front").xpos) * 1.8)
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
            if self.show_front_camera:
                self.add_front_camera_guide()
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

    def add_front_camera_guide(self) -> None:
        """Draw a schematic camera, optical axis and frustum only in the overview scene."""
        camera = self.env.data.cam("front")
        origin = camera.xpos.copy()
        rotation = camera.xmat.reshape(3, 3)
        blue = np.array([0.1, 0.75, 1.0, 1.0], dtype=np.float32)
        yellow = np.array([1.0, 0.6, 0.05, 1.0], dtype=np.float32)

        def geom(kind, size, position, matrix, color):
            item = self.scene.geoms[self.scene.ngeom]
            mj.mjv_initGeom(item, kind, np.asarray(size, dtype=float), position, matrix.ravel(), color)
            self.scene.ngeom += 1
            return item

        def line(start, end, color, arrow=False):
            kind = mj.mjtGeom.mjGEOM_ARROW if arrow else mj.mjtGeom.mjGEOM_LINE
            item = geom(kind, [0, 0, 0], origin, np.eye(3), color)
            mj.mjv_connector(item, kind, 0.003 if arrow else 2.0, start, end)

        if self.scene.ngeom + 11 > self.scene.maxgeom:
            return
        body = geom(
            mj.mjtGeom.mjGEOM_BOX, [0.025, 0.018, 0.012], origin + rotation[:, 2] * 0.014, rotation, blue
        )
        body.label = "front"
        geom(mj.mjtGeom.mjGEOM_SPHERE, [0.006, 0, 0], origin, np.eye(3), yellow)
        line(origin, origin - rotation[:, 2] * 0.22, yellow, arrow=True)
        depth = 0.14
        half_height = depth * np.tan(np.deg2rad(self.env.model.cam("front").fovy[0]) / 2)
        half_width = half_height * self.env.cfg.width / self.env.cfg.height
        corners = [
            origin + rotation @ np.array([x * half_width, y * half_height, -depth])
            for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]
        for index, corner in enumerate(corners):
            line(origin, corner, blue)
            line(corner, corners[(index + 1) % 4], blue)

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
