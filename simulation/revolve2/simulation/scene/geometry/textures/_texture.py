from dataclasses import dataclass

from ..._color import Color
from ._map_type import MapType


@dataclass(frozen=True, kw_only=True)
class Texture:
    """An abstract texture for geometric models."""

    name: str
    """name can be used to reference texture files or specific builtin textures."""
    primary_color: Color
    """_primary_color determines the color of the texture."""
    secondary_color: Color
    """_secondary_color facilitates more complex textures that use two colors."""
    map_type: MapType
    """_map_type represents how the texture is mapped to an object."""
    repeat: tuple[int, int]
    """_repeat allows you to set how often a texture should be repeated across the x and y axis (Only for 2d maps)."""
    size: tuple[int, int]
    """_size allows to set the size of a texture. The higher the size, the higher the resolution"""

    specular: float
    """Specular ranges between [0, 1]."""
    shininess: float
    """Shininess ranges between [0, 1]."""
    reflectance: float
    """Reflectance ranges between [0, 1]."""
    emission: float
