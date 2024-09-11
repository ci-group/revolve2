from enum import Enum


class MapType(Enum):
    """
    Enumerate different map types for textures.

    - "MAP2D": Maps the texture to a 2d surface.
    - "CUBE": Wraps the texture around a cube object.
    - "SKYBOX": Like "cube" but maps onto the inside of an object.
    """

    MAP2D = "2d"
    CUBE = "cube"
    SKYBOX = "skybox"
