from dataclasses import dataclass, field
from typing import List

from pyrr import Matrix33, Vector3
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
        return sum(collision.mass for collision in self.collisions)

    def center_of_mass(self) -> Vector3:
        return (
            sum(collision.mass * collision.position for collision in self.collisions)
            / self.mass()
        )

    def inertia_tensor(self) -> Matrix33:
        com = self.center_of_mass()
        inertia = Matrix33()

        for collision in self.collisions:
            inertia[0][0] += collision.mass * (
                1.0
                / 12.0
                * (collision.bounding_box.y ** 2 + collision.bounding_box.z ** 2)
                + (collision.position.y - com.y) ** 2
                + (collision.position.z - com.z) ** 2
            )
            inertia[1][1] += collision.mass * (
                1.0
                / 12.0
                * (collision.bounding_box.x ** 2 + collision.bounding_box.z ** 2)
                + (collision.position.x - com.x) ** 2
                + (collision.position.z - com.z) ** 2
            )
            inertia[2][2] += collision.mass * (
                1.0
                / 12.0
                * (collision.bounding_box.x ** 2 + collision.bounding_box.y ** 2)
                + (collision.position.x - com.x) ** 2
                + (collision.position.y - com.y) ** 2
            )

        return inertia
