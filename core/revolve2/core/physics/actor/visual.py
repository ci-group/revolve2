from dataclasses import dataclass
from typing import Tuple

from pyrr import Quaternion, Vector3


@dataclass
class Visual:
    name: str
    position: Vector3
    orientation: Quaternion
    model: str
    color: Tuple[float, float, float]
