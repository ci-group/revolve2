from dataclasses import dataclass, field

from ..._color import Color
from ._map_type import MapType
from ._texture import Texture


@dataclass(kw_only=True, frozen=True)
class Gradient(Texture):
    """A color gradient spanning over geometric models."""

    builtin: str = field(default="gradient")
    size: tuple[int, int] = field(default=(10, 10))
    repeat: tuple[int, int] = field(default=(1, 1))
    specular: float = field(default=0.5)
    shininess: float = field(default=0.5)
    reflectance: float = field(default=0.0)
    emission: float = field(default=0.0)
    map_type: MapType = field(default=MapType.CUBE)

    primary_color: Color
    secondary_color: Color
