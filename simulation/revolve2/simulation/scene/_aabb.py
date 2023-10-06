from dataclasses import dataclass

from pyrr import Vector3


@dataclass
class AABB:
    """An axis aligned bounding box."""

    size: Vector3
    """
    Sizes of the length of the bounding box.
    
    Not half of the box.
    """
