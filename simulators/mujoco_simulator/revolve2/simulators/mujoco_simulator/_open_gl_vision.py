import logging
import os
from typing import Any

import mujoco
import numpy as np
from mujoco import MjData, MjModel
from numpy.typing import NDArray

from ._abstraction_to_mujoco_mapping import CameraSensorMujoco
from ._render_backend import RenderBackend


class OpenGLVision:
    """A class to enable vision / camera sensors using OpenGl."""

    _open_gl_context: Any

    _mujoco_context: mujoco.MjrContext
    _mujoco_scene: mujoco.MjvScene

    _camera: mujoco.MjvCamera
    _image: NDArray[np.uint8]

    _video_options: mujoco.MjvOption
    _video_perturbations: mujoco.MjvPerturb

    def __init__(
        self,
        model: MjModel,
        camera: CameraSensorMujoco,
        headless: bool,
        open_gl_lib: RenderBackend,
        max_geometries: int = 10_000,
    ) -> None:
        """
        Initialize the vision object.

        Keep the following limitations in mind:
        - GLFW cannot work in a headless/ multithreaded environment.
        - EGL requires a GPU which may not be available in HPC contexts.
        - OSMESA in CPU only but does not scale to even moderate resolutions.

        :param model: The mujoco model.
        :param camera: The camera object.
        :param headless: Whether the simulation is run in headless mode.
        :param open_gl_lib: The type of library to use for OpenGL. [GLFW, EGL, OSMESA].
        :param max_geometries: The maximum amount of geometries allowed in a scene.
        """
        context = self.get_context(open_gl_lib)
        if headless:
            self._open_gl_context = context.gl_context(*camera.camera_size)
            self._open_gl_context.make_current()
            logging.debug(f"Initialized {OpenGLVision._open_gl_context=}")

        self._mujoco_context = mujoco.MjrContext(
            model, mujoco.mjtFontScale.mjFONTSCALE_150.value
        )
        self._viewport = mujoco.MjrRect(0, 0, *camera.camera_size)

        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self._camera.fixedcamid = camera.camera_id

        self._video_options = mujoco.MjvOption()
        self._mujoco_scene = mujoco.MjvScene(model, maxgeom=max_geometries)
        self._video_perturbations = mujoco.MjvPerturb()

        self._image = np.zeros(
            (*camera.camera_size, 3), dtype=np.uint8
        )  # Create an empty RGB-image.

    def process(self, model: MjModel, data: MjData) -> NDArray[np.uint8]:
        """
        Process the current state of the simulation and render it.

        :param model: The mujoco model.
        :param data: The mujoco data.
        :return: The rendered image (RGB format).
        """
        mujoco.mjv_updateScene(
            model,
            data,
            self._video_options,
            self._video_perturbations,
            self._camera,
            mujoco.mjtCatBit.mjCAT_ALL.value,
            self._mujoco_scene,
        )
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._mujoco_context)
        mujoco.mjr_render(self._viewport, self._mujoco_scene, self._mujoco_context)
        mujoco.mjr_readPixels(self._image, None, self._viewport, self._mujoco_context)
        return self._image

    @staticmethod
    def get_context(open_gl_lib: RenderBackend) -> Any:
        """
        Get the context to be used.

        :param open_gl_lib: The render backend to use.
        :return: The context type.
        :raises ValueError: If the backend for OpenGL is not known.
        """
        match open_gl_lib:
            case RenderBackend.GLFW:  # Does not work in multithread
                from mujoco.glfw import GLContext as GLFWContext

                gl_context = GLFWContext
            case RenderBackend.EGL:
                from mujoco.egl import GLContext as EGLContext

                gl_context = EGLContext
                os.environ["MUJOCO_GL"] = "egl"
            case RenderBackend.OSMESA:
                from mujoco.osmesa import GLContext as OSMESAContext

                gl_context = OSMESAContext
                os.environ["MUJOCO_GL"] = "osmesa"
            case _:
                raise ValueError(f"Unknown OpenGL backend {open_gl_lib}")
        return gl_context
