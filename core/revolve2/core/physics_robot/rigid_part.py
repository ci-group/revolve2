from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class RigidPart:
    position: Vector3
    orientation: Quaternion
    mass: float
    visual: str
    collision: str
