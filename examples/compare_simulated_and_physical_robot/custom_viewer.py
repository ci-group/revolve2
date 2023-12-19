import glfw
import imageio
import yaml
import mujoco_viewer
import mujoco
import numpy as np
import time
from typing import Any


class CustomMujocoViewer(mujoco_viewer.MujocoViewer):
    """Custom Viewer Object that allows for additional keyboard inputs."""

    _position: int = 0
    def _create_overlay(self) -> None:
        """ Create a Custom Overlay."""
        topleft = mujoco.mjtGridPos.mjGRID_TOPLEFT
        topright = mujoco.mjtGridPos.mjGRID_TOPRIGHT
        bottomleft = mujoco.mjtGridPos.mjGRID_BOTTOMLEFT
        bottomright = mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT

        def add_overlay(gridpos: int, text1: str, text2: str) -> None:
            """
            This function allows to add overlays.

            :param gridpos: The position on the grid.
            :param text1: Some text.
            :param text2: Additional text.
            """
            if gridpos not in self._overlay:
                self._overlay[gridpos] = ["", ""]
            self._overlay[gridpos][0] += text1 + "\n"
            self._overlay[gridpos][1] += text2 + "\n"

        add_overlay(
            topleft, "Switch camera (#cams = %d)" %
            (self.model.ncam + 1), "[Tab] (camera ID = %d)" %
            self.cam.fixedcamid)
        add_overlay(
            topleft,
            "T[r]ansparent",
            "On" if self._transparent else "Off")
        add_overlay(
            topleft,
            "[W]ireframe",
            "On" if self._wire_frame else "Off")
        add_overlay(
            topleft,
            "Con[V]ex Hull Rendering",
            "On" if self._convex_hull_rendering else "Off",
        )
        if self._paused is not None:
            if not self._paused:
                add_overlay(topleft, "Stop", "[Space]")
            else:
                add_overlay(topleft, "Start", "[Space]")
                add_overlay(
                    topleft,
                    "Advance simulation by one step",
                    "[right arrow]")
        add_overlay(topleft, "Toggle geomgroup visibility (0-5)",
                    ",".join(["On" if g else "Off" for g in self.vopt.geomgroup]))
        add_overlay(
            topleft,
            "Referenc[e] frames",
            mujoco.mjtFrame(self.vopt.frame).name)
        add_overlay(topleft, "[H]ide Menus", "")
        if self._image_idx > 0:
            fname = self._image_path % (self._image_idx - 1)
            add_overlay(topleft, "Cap[t]ure frame", "Saved as %s" % fname)
        else:
            add_overlay(topleft, "Cap[t]ure frame", "")
        add_overlay(topleft, "Iterate position", "[k]")


        add_overlay(
            bottomleft, "FPS", "%d%s" %
            (1 / self._time_per_render, ""))
        add_overlay(
            bottomleft, "Solver iterations", str(
                self.data.solver_iter + 1))
        add_overlay(
            bottomleft, "Step", str(
                round(
                    self.data.time / self.model.opt.timestep)))
        add_overlay(bottomleft, "timestep", "%.5f" % self.model.opt.timestep)
        add_overlay(bottomleft, "position", str(self._position+1))

    def _key_callback(self, window: Any, key: Any, scancode: Any, action: Any, mods: Any) -> None:
        """
        Add custom Key Callback.
        Don't change parameters unless you know what you are doing.

        :param window: The window.
        :param key: The key pressed.
        :param scancode: The Scancode.
        :param action: The Action.
        :param mods: The Mods.
        """
        if action != glfw.RELEASE:
            if key == glfw.KEY_LEFT_ALT:
                self._hide_menus = False
            return
        # Switch cameras
        elif key == glfw.KEY_TAB:
            self.cam.fixedcamid += 1
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            if self.cam.fixedcamid >= self.model.ncam:
                self.cam.fixedcamid = -1
                self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        # Pause simulation
        elif key == glfw.KEY_SPACE and self._paused is not None:
            self._paused = not self._paused
        # Advances simulation by one step.
        elif key == glfw.KEY_RIGHT and self._paused is not None:
            self._advance_by_one_step = True
        # Capture screenshot
        elif key == glfw.KEY_T:
            img = np.zeros(
                (glfw.get_framebuffer_size(
                    self.window)[1], glfw.get_framebuffer_size(
                    self.window)[0], 3), dtype=np.uint8)
            mujoco.mjr_readPixels(img, None, self.viewport, self.ctx)
            imageio.imwrite(self._image_path % self._image_idx, np.flipud(img))
            self._image_idx += 1
        # Make transparent
        elif key == glfw.KEY_R:
            self._transparent = not self._transparent
            if self._transparent:
                self.model.geom_rgba[:, 3] /= 5.0
            else:
                self.model.geom_rgba[:, 3] *= 5.0
        # Toggle Graph overlay
        elif key == glfw.KEY_G:
            self._hide_graph = not self._hide_graph
        # Convex-Hull rendering
        elif key == glfw.KEY_V:
            self._convex_hull_rendering = not self._convex_hull_rendering
            self.vopt.flags[
                mujoco.mjtVisFlag.mjVIS_CONVEXHULL
            ] = self._convex_hull_rendering
        # Wireframe Rendering
        elif key == glfw.KEY_W:
            self._wire_frame = not self._wire_frame
            self.scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = self._wire_frame
        elif key == glfw.KEY_K:
            self.increment_position()
        # Geom group visibility
        elif key in (glfw.KEY_0, glfw.KEY_1, glfw.KEY_2, glfw.KEY_3, glfw.KEY_4, glfw.KEY_5):
            self.vopt.geomgroup[key - glfw.KEY_0] ^= 1
        elif key == glfw.KEY_S and mods == glfw.MOD_CONTROL:
            cam_config = {
                "type": self.cam.type,
                "fixedcamid": self.cam.fixedcamid,
                "trackbodyid": self.cam.trackbodyid,
                "lookat": self.cam.lookat.tolist(),
                "distance": self.cam.distance,
                "azimuth": self.cam.azimuth,
                "elevation": self.cam.elevation
            }
            try:
                with open(self.CONFIG_PATH, "w") as f:
                    yaml.dump(cam_config, f)
                print("Camera config saved at {}".format(self.CONFIG_PATH))
            except Exception as e:
                print(e)
        # Quit
        if key == glfw.KEY_ESCAPE:
            print("Pressed ESC")
            print("Quitting.")
            glfw.set_window_should_close(self.window, True)
        return

    def render(self) -> int | None:
        """
        Rendering our scene.

        :return: A cycle position if applicable.
        """
        if self.render_mode == 'offscreen':
            raise NotImplementedError(
                "Use 'read_pixels()' for 'offscreen' mode.")
        if not self.is_alive:
            raise Exception(
                "GLFW window does not exist but you tried to render.")
        if glfw.window_should_close(self.window):
            self.close()
            return

        def update() -> None:
            """Updating the render."""
            self._create_overlay()  # fill overlay items

            render_start = time.time()

            width, height = glfw.get_framebuffer_size(self.window)
            self.viewport.width, self.viewport.height = width, height

            with self._gui_lock:
                # update scene
                mujoco.mjv_updateScene(
                    self.model,
                    self.data,
                    self.vopt,
                    self.pert,
                    self.cam,
                    mujoco.mjtCatBit.mjCAT_ALL.value,
                    self.scn)
                # marker items
                for marker in self._markers:
                    self._add_marker_to_scene(marker)
                # render
                mujoco.mjr_render(self.viewport, self.scn, self.ctx)
                # overlay items
                for gridpos, [t1, t2] in self._overlay.items():
                    menu_positions = [mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                      mujoco.mjtGridPos.mjGRID_BOTTOMLEFT]
                    if gridpos in menu_positions and self._hide_menus:
                        continue

                    mujoco.mjr_overlay(
                        mujoco.mjtFontScale.mjFONTSCALE_150,
                        gridpos,
                        self.viewport,
                        t1,
                        t2,
                        self.ctx)

                # handle figures
                if not self._hide_graph:
                    for idx, fig in enumerate(self.figs):
                        width_adjustment = width % 4
                        x = int(3 * width / 4) + width_adjustment
                        y = idx * int(height / 4)
                        viewport = mujoco.MjrRect(
                            x, y, int(width / 4), int(height / 4))

                        has_lines = len([i for i in fig.linename if i != b''])
                        if has_lines:
                            mujoco.mjr_figure(viewport, fig, self.ctx)

                glfw.swap_buffers(self.window)
            glfw.poll_events()
            self._time_per_render = 0.9 * self._time_per_render + \
                0.1 * (time.time() - render_start)

            # clear overlay
            self._overlay.clear()

        if self._paused:
            while self._paused:
                update()
                if glfw.window_should_close(self.window):
                    self.close()
                    break
                if self._advance_by_one_step:
                    self._advance_by_one_step = False
                    break
        else:
            self._loop_count += self.model.opt.timestep / \
                (self._time_per_render * self._run_speed)
            if self._render_every_frame:
                self._loop_count = 1
            while self._loop_count > 0:
                update()
                self._loop_count -= 1

        # clear markers
        self._markers[:] = []

        # apply perturbation (should this come before mj_step?)
        self.apply_perturbations()
        return self._position

    def increment_position(self) -> None:
        """Increment our cycle position."""
        self._position = (self._position + 1) % 5
