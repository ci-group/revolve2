from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class Visual:
    """Visual-only part of an actor."""

    name: str
    position: Vector3
    orientation: Quaternion
    model: str
    color: tuple[float, float, float]
