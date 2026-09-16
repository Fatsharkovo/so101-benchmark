"""Render a few frames on a real or Xvfb display, capture only our own window."""

from __future__ import annotations

import os

os.environ["MUJOCO_GL"] = "glfw"


def main():
    import mujoco
    import numpy as np
    from PIL import Image

    from lerobot_env_so101.config import SimConfig
    from lerobot_env_so101.env import SO101Env
    from lerobot_env_so101.viewer import ThreeViewWindow

    env = SO101Env(cfg=SimConfig(width=320, height=240, render_backend="glfw"))
    viewer = None
    try:
        env.reset(seed=0)
        viewer = ThreeViewWindow(env)
        for _ in range(5):
            env.step(env.applied_action)
            viewer.draw({}, "validation")
        # Draw once more to the back buffer, before swapping.
        width, height = viewer.glfw.get_framebuffer_size(viewer.window)
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        viewer.glfw.make_context_current(viewer.window)
        mujoco.mjr_readPixels(rgb, None, mujoco.MjrRect(0, 0, width, height), viewer.context)
        Image.fromarray(rgb[::-1]).save("/tmp/so101-three-view-window.png")
        print("Three-view GLFW window rendered successfully")
    finally:
        if viewer:
            viewer.close()
        env.close()


if __name__ == "__main__":
    main()
