from dataclasses import dataclass

from pyrr import Vector3


@dataclass
class BoundingBox:
    size: Vector3  # size of each side. not half.
    offset: Vector3  # offset from origin
