from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class Collision:
    """A collision box."""

    """Name of the collision."""
    name: str

    """Position of the collision."""
    position: Vector3

    """Orientation of the collision."""
    orientation: Quaternion

    """
    Mass of the collision.

    This the absolute mass, irrespective of the size of the bounding box.
    """
    mass: float

    """
    Box describing the collision.

    Sizes of the length of the bounding box.
    Not half of the box.
    """
    bounding_box: Vector3
