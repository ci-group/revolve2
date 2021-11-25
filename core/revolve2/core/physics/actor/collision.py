from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class Collision:
    name: str
    position: Vector3
    orientation: Quaternion
    mass: float
    bounding_box: Vector3
