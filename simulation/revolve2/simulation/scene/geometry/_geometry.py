from dataclasses import dataclass

from .._pose import Pose
from .textures import Texture


@dataclass(kw_only=True)
class Geometry:
    """
    Geometry describing part of a rigid body shape.

    Currently always a box.
    """

    pose: Pose
    """Pose of the geometry."""

    mass: float
    """
    Mass of the geometry.

    This the absolute mass, irrespective of the size of the bounding box.
    """

    texture: Texture
    """Texture when rendering this geometry."""
