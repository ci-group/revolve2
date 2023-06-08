from dataclasses import dataclass

from pyrr import Vector3


@dataclass
class BoundingBox:
    """A box with a position."""

    """Size of each side, not half."""
    size: Vector3

    """Offset from origin."""
    offset: Vector3
