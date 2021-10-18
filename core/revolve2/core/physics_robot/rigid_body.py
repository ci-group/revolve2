from dataclasses import dataclass, field
from typing import List

from pyrr import Vector3, matrix33
from pyrr.objects.quaternion import Quaternion

from .collision import Collision
from .visual import Visual


@dataclass
class RigidBody:
    name: str
    position: Vector3
    orientation: Quaternion
    collisions: List[Collision] = field(default_factory=list, init=False)
    visuals: List[Visual] = field(default_factory=list, init=False)

    def mass(self) -> float:
        return sum(part.mass for part in self.collisions)

    def center_of_mass(self) -> Vector3:
        return sum(part.mass * part.position for part in self.collisions) / self.mass()

    def inertia_tensor(self) -> matrix33:
        raise NotImplementedError()  # TODO
