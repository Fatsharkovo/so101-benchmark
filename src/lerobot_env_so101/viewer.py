from __future__ import annotations

import numpy as np


class ThreeViewWindow:
    """Third-person MuJoCo camera plus the exact front/wrist observation buffers."""

    def __init__(self, env):
        import glfw
        import mujoco

        self.glfw, self.mj, self.env = glfw, mujoco, env
        if not glfw.init():
            raise RuntimeError("GLFW cannot open a display; use display=false for headless evaluation")
        # MuJoCo's offscreen GLFW context leaves the global VISIBLE hint disabled.
        glfw.window_hint(glfw.VISIBLE, glfw.TRUE)
        self.window = glfw.create_window(1280, 720, "SO-101 Benchmark", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Unable to create benchmark window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(0)
        self.context = mujoco.MjrContext(env.model, mujoco.mjtFontScale.mjFONTSCALE_100)
        self.scene = mujoco.MjvScene(env.model, maxgeom=5000)
        self.option = mujoco.MjvOption()
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.15, 0, 0.07]
        self.camera.distance = 0.75
        self.camera.azimuth = 135
        self.camera.elevation = -35
        self.last_cursor = None
        glfw.set_cursor_pos_callback(self.window, self._mouse)
        glfw.set_scroll_callback(self.window, self._scroll)
        glfw.set_key_callback(self.window, self._key)

    def _key(self, window, key, scancode, action, mods):
        if key == self.glfw.KEY_ESCAPE and action == self.glfw.PRESS:
            self.glfw.set_window_should_close(window, True)

    def _mouse(self, window, x, y):
        previous, self.last_cursor = self.last_cursor, (x, y)
        if previous is None:
            return
        left = self.glfw.get_mouse_button(window, self.glfw.MOUSE_BUTTON_LEFT)
        right = self.glfw.get_mouse_button(window, self.glfw.MOUSE_BUTTON_RIGHT)
        if not (left or right):
            return
        height = self.glfw.get_window_size(window)[1]
        action = self.mj.mjtMouse.mjMOUSE_MOVE_H if right else self.mj.mjtMouse.mjMOUSE_ROTATE_H
        self.mj.mjv_moveCamera(
            self.env.model,
            action,
            (x - previous[0]) / height,
            (y - previous[1]) / height,
            self.scene,
            self.camera,
        )

    def _scroll(self, window, x, y):
        self.mj.mjv_moveCamera(
            self.env.model, self.mj.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * y, self.scene, self.camera
        )

    def draw(self, info, mode):
        mj, glfw, env = self.mj, self.glfw, self.env
        glfw.make_context_current(self.window)
        width, height = glfw.get_framebuffer_size(self.window)
        third = mj.MjrRect(0, 0, width * 2 // 3, height)
        mj.mjv_updateScene(
            env.model, env.data, self.option, None, self.camera, mj.mjtCatBit.mjCAT_ALL, self.scene
        )
        mj.mjr_render(third, self.scene, self.context)
        for i, name in enumerate(("front", "wrist")):
            w, h = width // 3, height // 2
            frame = env.frames[name]
            # Nearest-neighbour presentation only; policy/recording buffers remain untouched.
            ys = np.minimum(np.arange(h) * frame.shape[0] // h, frame.shape[0] - 1)
            xs = np.minimum(np.arange(w) * frame.shape[1] // w, frame.shape[1] - 1)
            pixels = np.ascontiguousarray(frame[ys[:, None], xs][::-1])
            rect = mj.MjrRect(width * 2 // 3, (1 - i) * h, w, h)
            mj.mjr_drawPixels(pixels.reshape(-1), None, rect, self.context)
            mj.mjr_overlay(
                mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT, rect, name, "", self.context
            )
        label = f"{env.task} | {mode} | step {env.step_index} | success {info.get('is_success', False)}"
        mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT, third, label, "", self.context)
        glfw.swap_buffers(self.window)
        glfw.poll_events()
        return not glfw.window_should_close(self.window)

    def close(self):
        self.glfw.make_context_current(self.window)
        self.context.free()
        self.glfw.destroy_window(self.window)
        # MuJoCo's offscreen GLFW renderer still owns a hidden window. Its lifetime
        # is independent, so do not globally terminate GLFW here.
