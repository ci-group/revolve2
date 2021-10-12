from typing import List

from pyrr import Vector3, matrix33
from pyrr.objects.quaternion import Quaternion

from .rigid_part import RigidPart


class RigidBody:
    name: str
    position: Vector3
    orientation: Quaternion
    parts: List[RigidPart]

    def __init__(self, name: str, position: Vector3, orientation: Quaternion):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.parts = []

    def mass(self) -> float:
        return sum(part.mass for part in self.parts)

    def center_of_mass(self) -> Vector3:
        return sum(part.mass * part.position for part in self.parts) / self.mass()

    def inertia_tensor(self) -> matrix33:
        raise NotImplementedError()  # TODO
