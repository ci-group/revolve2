from dataclasses import dataclass, field
from typing import List

from pyrr import Matrix33, Vector3
from pyrr.objects.quaternion import Quaternion

from ._collision import Collision
from ._visual import Visual


@dataclass
class RigidBody:
    name: str
    position: Vector3
    orientation: Quaternion
    static_friction: float
    dynamic_friction: float
    collisions: List[Collision] = field(default_factory=list, init=False)
    visuals: List[Visual] = field(default_factory=list, init=False)

    def mass(self) -> float:
        return sum(collision.mass for collision in self.collisions)

    def center_of_mass(self) -> Vector3:
        """
        center of mass in local reference frame of this rigid body.
        """

        return (
            sum(collision.mass * collision.position for collision in self.collisions)
            / self.mass()
        )

    def inertia_tensor(self) -> Matrix33:
        """
        intertia tensor in local reference frame of this rigid body.
        """

        com = self.center_of_mass()
        inertia = Matrix33()

        for collision in self.collisions:
            # calculate inertia in local coordinates
            local_inertia = Matrix33()
            local_inertia[0][0] += (
                collision.mass
                * (collision.bounding_box.y**2 + collision.bounding_box.z**2)
                / 12.0
            )
            local_inertia[1][1] += (
                collision.mass
                * (collision.bounding_box.x**2 + collision.bounding_box.z**2)
                / 12.0
            )
            local_inertia[2][2] += (
                collision.mass
                * (collision.bounding_box.x**2 + collision.bounding_box.y**2)
                / 12.0
            )

            # convert to global coordinates
            translation = Matrix33()
            translation[0][0] += collision.mass * (
                (collision.position.y - com.y) ** 2
                + (collision.position.z - com.z) ** 2
            )
            translation[1][1] += collision.mass * (
                (collision.position.x - com.x) ** 2
                + (collision.position.z - com.z) ** 2
            )
            translation[2][2] += collision.mass * (
                (collision.position.x - com.x) ** 2
                + (collision.position.y - com.y) ** 2
            )

            global_inertia = local_inertia * collision.orientation + translation

            # add to rigid body inertia tensor
            inertia += global_inertia

        return inertia
