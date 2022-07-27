from dataclasses import dataclass

from pyrr import Vector3


# TODO replace with pyrr aabb
# TODO merge with collision?
@dataclass
class BoundingBox:
    """A box with a position."""

    """Size of each side, not half."""
    size: Vector3

    """Offset from origin."""
    offset: Vector3
