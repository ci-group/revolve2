from dataclasses import dataclass, field

from .._color import Color
from ..vector2 import Vector2
from ._geometry import Geometry
from .textures import Checker, MapType, Texture


@dataclass(kw_only=True)
class GeometryPlane(Geometry):
    """A flat plane geometry."""

    size: Vector2
    texture: Texture = field(
        default_factory=lambda: Checker(
            primary_color=Color(100, 100, 100, 255),
            secondary_color=Color(120, 120, 120, 255),
            map_type=MapType.MAP2D,
        )
    )
