from dataclasses import dataclass
from typing import List

from .joint import Joint
from .rigid_body import RigidBody


@dataclass
class PhysicsRobot:
    bodies: List[RigidBody]
    joints: List[Joint]
