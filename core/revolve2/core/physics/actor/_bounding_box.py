from dataclasses import dataclass

from pyrr import Vector3


# TODO replace with pyrr aabb
@dataclass
class BoundingBox:
    size: Vector3  # size of each side. not half.
    offset: Vector3  # offset from origin
