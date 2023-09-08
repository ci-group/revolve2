from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class BodyState:
    """State of a modular robot body."""

    core_position: Vector3
    core_orientation: Quaternion
