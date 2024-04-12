from dataclasses import dataclass, field

from ..._color import Color
from ._map_type import MapType
from ._texture_reference import TextureReference


@dataclass(frozen=True, kw_only=True)
class Texture:
    """An abstract texture for geometric models."""

    reference: TextureReference | None = field(default=None)
    """Reference the specific type of texture. If no reference is given, the base_color will be used on the Object."""
    base_color: Color = field(default_factory=lambda: Color(255, 255, 255, 255))
    """Determines the base color of a material and its alpha value."""
    primary_color: Color = field(default_factory=lambda: Color(0, 0, 0, 0))
    """Determines the primary color of the texture."""
    secondary_color: Color = field(default_factory=lambda: Color(0, 0, 0, 0))
    """facilitates more complex textures that use two colors."""
    map_type: MapType = field(default=MapType.CUBE)
    """Represents how the texture is mapped to an object."""
    repeat: tuple[int, int] = field(default=(1, 1))
    """Allows you to set how often a texture should be repeated across the x and y axis (Only for 2d maps)."""
    size: tuple[int, int] = field(default=(100, 100))
    """Allows to set the size of a texture. The higher the size, the higher the resolution"""

    specular: float = field(default=0.5)
    """Specular ranges between [0, 1]."""
    shininess: float = field(default=0.5)
    """Shininess ranges between [0, 1]."""
    reflectance: float = field(default=0.0)
    """Reflectance ranges between [0, 1]."""
    emission: float = field(default=0.0)
    """Allows Objects to emit light [0, inf)."""
