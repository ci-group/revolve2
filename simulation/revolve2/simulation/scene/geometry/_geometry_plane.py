from dataclasses import dataclass, field

from .._color import Color
from ..vector2 import Vector2
from ._geometry import Geometry


@dataclass(kw_only=True)
class GeometryPlane(Geometry):
    """A flat plane geometry."""

    size: Vector2
    color: Color = field(default_factory=lambda: Color(100, 100, 100, 255))
