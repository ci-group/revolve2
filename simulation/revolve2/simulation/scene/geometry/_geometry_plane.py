from dataclasses import dataclass, field

from .._color import Color
from ..vector2 import Vector2
from ._geometry import Geometry
from .textures import Checker, Texture


@dataclass(kw_only=True)
class GeometryPlane(Geometry):
    """A flat plane geometry."""

    size: Vector2
    texture: Texture = field(
        default_factory=lambda: Checker(
            Color(100, 100, 100, 255), Color(120, 120, 120, 255), map_type="2d"
        )
    )
