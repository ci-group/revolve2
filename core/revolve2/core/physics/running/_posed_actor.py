from dataclasses import dataclass
from typing import List

from pyrr import Quaternion, Vector3

from ..actor import Actor


@dataclass
class PosedActor:
    """An actor with a position, orientation, and degree of freedom state."""

    actor: Actor
    position: Vector3
    orientation: Quaternion
    dof_states: List[float]
