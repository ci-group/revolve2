from enum import Enum


class RenderBackend(Enum):
    """Enumerator for rendering backend libraries."""

    GLFW = "glfw"
    EGL = "egl"
    OSMESA = "osmesa"
