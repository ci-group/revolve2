from dataclasses import dataclass

from ..._color import Color
from ._map_type import MapType


@dataclass(frozen=True, kw_only=True)
class Texture:
    """An abstract texture for geometric models."""

    builtin: str
    """
    builtin can be used to reference specific builtin textures in the target simulator.
    
    Mujoco has the following builtin textures available: 
    - flat
    - checker
    - gradient
    """
    base_color: Color
    """Determines the base color of a material and its alpha value."""
    primary_color: Color
    """Determines the primary color of the texture."""
    secondary_color: Color
    """facilitates more complex textures that use two colors."""
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
    """Allows Objects to emit light [0, inf)."""
