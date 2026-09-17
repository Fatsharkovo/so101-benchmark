import os

import pytest


@pytest.mark.skipif(os.environ.get("SO101_TEST_GUI") != "1", reason="Set SO101_TEST_GUI=1 on a GLFW display")
def test_windows_survive_model_rebuild_and_space_is_debounced():
    import glfw
    import numpy as np

    from lerobot_env_so101.config import SimConfig
    from lerobot_env_so101.env import SO101Env
    from lerobot_env_so101.teleop_viewer import SeparateViews

    env = SO101Env("stack_blue_on_red", SimConfig(render_backend="glfw", width=64, height=64))
    viewer = None
    try:
        env.reset(seed=0)
        viewer = SeparateViews(env)
        windows = [viewer.window, *(window for _, window, _ in viewer.extra)]
        ids = [glfw.get_x11_window(window) for window in windows]
        viewer.camera.azimuth = 120
        glfw.set_window_size(viewer.window, 900, 600)
        for seed in range(3):
            env.reset(seed=seed)
            viewer.draw({}, "WAITING | Saved episodes: 0 | Space: start recording")
            assert [glfw.get_x11_window(window) for window in windows] == ids
            assert glfw.get_window_size(viewer.window) == (900, 600)
            assert viewer.camera.azimuth == 120
            assert all(glfw.get_window_attrib(window, glfw.VISIBLE) for window in windows)
        images = {name: pixels.copy() for name, pixels in env.frames.items()}
        viewer.show_front_camera = False
        viewer.draw({}, "guide disabled")
        count = viewer.scene.ngeom
        viewer.show_front_camera = True
        viewer.draw({}, "guide enabled")
        assert viewer.scene.ngeom == count + 11
        lens = viewer.scene.geoms[count + 1]
        np.testing.assert_allclose(lens.pos, env.data.cam("front").xpos)
        for name, pixels in images.items():
            np.testing.assert_array_equal(env.frames[name], pixels)
        viewer._key(viewer.window, glfw.KEY_SPACE, 0, glfw.REPEAT, 0)
        assert not viewer.start_requested
        viewer._key(viewer.window, glfw.KEY_SPACE, 0, glfw.PRESS, 0)
        assert viewer.start_requested
    finally:
        if viewer:
            viewer.close()
        env.close()
