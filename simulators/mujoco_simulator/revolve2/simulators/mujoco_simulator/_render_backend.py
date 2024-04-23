from enum import Enum, auto


class RenderBackend(Enum):
    """Enumerator for rendering backend libraries."""

    GLFW = auto()
    EGL = auto()
    OSMESA = auto()
