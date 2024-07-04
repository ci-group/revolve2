"""A custom viewer for mujoco with additional features."""

from enum import Enum
from typing import Any

import glfw
import mujoco
import mujoco_viewer
from revolve2.simulation.simulator import Viewer

from .._render_backend import RenderBackend


class CustomMujocoViewerMode(Enum):
    """
    Enumerate different viewer modes for the CustomMujocoViewer.

    - CLASSIC mode gives an informative interface for regular simulations.
    - MANUAL mode gives a cut down interface, specific for targeting robot movement manually.
    """

    CLASSIC = "classic"
    MANUAL = "manual"


class CustomMujocoViewer(Viewer, mujoco_viewer.MujocoViewer):  # type: ignore
    """
    Custom Viewer Object that allows for additional keyboard inputs.

    We need the type ignore since the mujoco_viewer library is not typed properly and therefor the MujocoViewer class cant be resolved.
    """

    _convex_hull_rendering: bool
    _transparent: bool
    _paused: bool
    _hide_graph: bool
    _wire_frame: bool
    _time_per_render: float
    _loop_count: int
    _mujoco_version: tuple[int, ...]

    _viewer_mode: CustomMujocoViewerMode
    _advance_by_one_step: bool
    _position: int

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        backend: RenderBackend,
        width: int | None = None,
        height: int | None = None,
        start_paused: bool = False,
        render_every_frame: bool = False,
        hide_menus: bool = False,
        mode: CustomMujocoViewerMode = CustomMujocoViewerMode.CLASSIC,
        **_: Any,
    ):
        """
        Initialize the Viewer.

        :param model: The mujoco models.
        :param data: The mujoco data.
        :param backend: The backend for rendering.
        :param width: The width of the viewer (optional, defaults to screen width)
        :param height: The height of the viewer (optional, defaults to screen height)
        :param start_paused: If the simulation starts paused or not.
        :param render_every_frame: If every frame is rendered or not.
        :param hide_menus: Start with hidden menus?
        :param mode: The mode of the viewer (classic, manual).
        :param _: Some unused kwargs.
        """
        match backend:
            case RenderBackend.EGL:
                glfw.window_hint(glfw.CONTEXT_CREATION_API, glfw.EGL_CONTEXT_API)
            case RenderBackend.OSMESA:
                glfw.window_hint(glfw.CONTEXT_CREATION_API, glfw.OSMESA_CONTEXT_API)
            case _:  # By default, we are using GLFW.
                pass

        super().__init__(
            model,
            data,
            mode="window",
            title="custom-mujoco-viewer",
            width=width,
            height=height,
            hide_menus=hide_menus,
        )

        self._viewer_mode = mode
        self._position = 0
        self._paused = start_paused
        self._mujoco_version = tuple(map(int, mujoco.__version__.split(".")))
        self._render_every_frame = render_every_frame

    def _add_overlay(self, gridpos: int, text1: str, text2: str) -> None:
        """
        Add overlays.

        :param gridpos: The position on the grid.
        :param text1: Some text.
        :param text2: Additional text.
        """
        if gridpos not in self._overlay:
            self._overlay[gridpos] = ["", ""]
        self._overlay[gridpos][0] += text1 + "\n"
        self._overlay[gridpos][1] += text2 + "\n"

    def _create_overlay(self) -> None:
        """Create a Custom Overlay."""
        topleft = mujoco.mjtGridPos.mjGRID_TOPLEFT
        # topright = mujoco.mjtGridPos.mjGRID_TOPRIGHT
        bottomleft = mujoco.mjtGridPos.mjGRID_BOTTOMLEFT
        # bottomright = mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT

        match self._viewer_mode.value:
            case "manual":
                self._add_overlay(topleft, "Iterate position", "[K]")
                self._add_overlay(bottomleft, "position", str(self._position + 1))
            case "classic":
                self._add_overlay(
                    topleft, "[C]ontact forces", "On" if self._contacts else "Off"
                )
                self._add_overlay(topleft, "[J]oints", "On" if self._joints else "Off")
                self._add_overlay(
                    topleft, "[G]raph Viewer", "Off" if self._hide_graph else "On"
                )
                self._add_overlay(
                    topleft, "[I]nertia", "On" if self._inertias else "Off"
                )
                self._add_overlay(
                    topleft, "Center of [M]ass", "On" if self._com else "Off"
                )
            case _:
                print("Didnt reach anything with mode: " + self._viewer_mode.value)

        """These are default overlays, only change if you know what you are doing."""
        if self._render_every_frame:
            self._add_overlay(topleft, "", "")
        else:
            self._add_overlay(
                topleft,
                "Run speed = %.3f x real time" % self._run_speed,
                "[S]lower, [F]aster",
            )
        self._add_overlay(
            topleft, "Ren[d]er every frame", "On" if self._render_every_frame else "Off"
        )
        self._add_overlay(
            topleft,
            "Switch camera (#cams = %d)" % (self.model.ncam + 1),
            "[Tab] (camera ID = %d)" % self.cam.fixedcamid,
        )

        self._add_overlay(topleft, "Shad[O]ws", "On" if self._shadows else "Off")
        self._add_overlay(
            topleft, "T[r]ansparent", "On" if self._transparent else "Off"
        )
        self._add_overlay(topleft, "[W]ireframe", "On" if self._wire_frame else "Off")
        self._add_overlay(
            topleft,
            "Con[V]ex Hull Rendering",
            "On" if self._convex_hull_rendering else "Off",
        )
        if self._paused is not None:
            if not self._paused:
                self._add_overlay(topleft, "Stop", "[Space]")
            else:
                self._add_overlay(topleft, "Start", "[Space]")
                self._add_overlay(
                    topleft, "Advance simulation by one step", "[right arrow]"
                )
        self._add_overlay(
            topleft,
            "Toggle geomgroup visibility (0-5)",
            ",".join(["On" if g else "Off" for g in self.vopt.geomgroup]),
        )
        self._add_overlay(
            topleft, "Referenc[e] frames", mujoco.mjtFrame(self.vopt.frame).name
        )
        self._add_overlay(topleft, "[H]ide Menus", "")
        if self._image_idx > 0:
            fname = self._image_path % (self._image_idx - 1)
            self._add_overlay(topleft, "Cap[t]ure frame", "Saved as %s" % fname)
        else:
            self._add_overlay(topleft, "Cap[t]ure frame", "")

        self._add_overlay(bottomleft, "FPS", "%d%s" % (1 / self._time_per_render, ""))

        if self._mujoco_version >= (3, 0, 0):
            self._add_overlay(
                bottomleft, "Max solver iters", str(max(self.data.solver_niter) + 1)
            )
        else:
            self._add_overlay(
                bottomleft, "Solver iterations", str(self.data.solver_iter + 1)
            )

        self._add_overlay(
            bottomleft, "Step", str(round(self.data.time / self.model.opt.timestep))
        )
        self._add_overlay(bottomleft, "timestep", "%.5f" % self.model.opt.timestep)

    def _key_callback(
        self, window: Any, key: Any, scancode: Any, action: Any, mods: Any
    ) -> None:
        """
        Add custom Key Callback.

        :param window: The window.
        :param key: The key pressed.
        :param scancode: The Scancode.
        :param action: The Action.
        :param mods: The Mods.
        """
        super()._key_callback(window, key, scancode, action, mods)
        if action != glfw.RELEASE:
            if key == glfw.KEY_LEFT_ALT:
                self._hide_menus = False
        else:
            match key:
                case glfw.KEY_K:  # Increment cycle position
                    self._increment_position()
                case _:
                    pass

    def current_viewport_size(self) -> tuple[int, int]:
        """
        Grabs the *current* viewport size (and updates the cached values).

        :return: the viewport size
        """
        self.viewport.width, self.viewport.height = glfw.get_framebuffer_size(
            self.window
        )
        return self.viewport.width, self.viewport.height

    def render(self) -> int | None:
        """
        Render the scene.

        :return: A cycle position if applicable.
        """
        super().render()
        if self._viewer_mode == CustomMujocoViewerMode.MANUAL:
            return self._position
        return None

    def _increment_position(self) -> None:
        """Increment our cycle position."""
        self._position = (self._position + 1) % 5

    def close_viewer(self) -> None:
        """Close the viewer."""
        self.close()

    @property
    def context(self) -> mujoco.MjrContext:
        """
        Get the context.

        :returns: The context.
        """
        return self.ctx

    @property
    def view_port(self) -> mujoco.MjrRect:
        """
        Get the view_port.

        :returns: The viewport.
        """
        return self.viewport

    @property
    def can_record(self) -> bool:
        """
        Return True.

        :returns: True.
        """
        return True
