from dataclasses import dataclass

from .._aabb import AABB
from ._geometry import Geometry
from pyrr import Vector3

@dataclass(kw_only=True)
class GeometryCylinder(Geometry):
    """Cylinder geometry."""

    radius: float
    """The radius of the cylinder."""

    length: float
    """The length of the cylinder."""
