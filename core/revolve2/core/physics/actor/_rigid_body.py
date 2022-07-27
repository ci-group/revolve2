from dataclasses import dataclass, field
from typing import List

from pyrr import Matrix33, Vector3
from pyrr.objects.quaternion import Quaternion

from ._collision import Collision
from ._visual import Visual


@dataclass
class RigidBody:
    """A collection of collision objects with an orientation, position and friction parameters."""

    name: str
    position: Vector3
    orientation: Quaternion
    static_friction: float
    dynamic_friction: float
    collisions: List[Collision] = field(default_factory=list, init=False)
    visuals: List[Visual] = field(default_factory=list, init=False)

    def mass(self) -> float:
        """Get the center of mass.

        :returns: The center of mass.
        """
        return sum(collision.mass for collision in self.collisions)

    def center_of_mass(self) -> Vector3:
        """
        Calculate the center of mass in the local reference frame of this rigid body.

        :returns: The center of mass.
        """
        return (
            sum(collision.mass * collision.position for collision in self.collisions)
            / self.mass()
        )

    def inertia_tensor(self) -> Matrix33:
        """
        Calculate the inertia tensor in the local reference frame of this rigid body.

        :returns: The inertia tensor.
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

            ori_as_mat = self._quaternion_to_rotation_matrix(collision.orientation)
            global_inertia = (
                ori_as_mat * local_inertia * ori_as_mat.transpose() + translation
            )
            # add to rigid body inertia tensor
            inertia += global_inertia

        return inertia

    @staticmethod
    def _quaternion_to_rotation_matrix(quat: Quaternion) -> Matrix33:
        # https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/

        q0 = quat.x
        q1 = quat.y
        q2 = quat.z
        q3 = quat.w

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        return Matrix33([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
