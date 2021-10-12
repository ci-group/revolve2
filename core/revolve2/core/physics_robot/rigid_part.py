from dataclasses import dataclass
from typing import Tuple

from pyrr import Quaternion, Vector3


@dataclass
class RigidPart:
    name: str
    position: Vector3
    orientation: Quaternion
    mass: float
    visual_model: str
    visual_color: Tuple[float, float, float]
    collision_size: Vector3
