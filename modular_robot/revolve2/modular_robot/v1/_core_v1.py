import math

from pyrr import Quaternion, Vector3
from revolve2.simulation.actor import Actor, Collision, Color, RigidBody

from .._common import Core
from .._right_angles import RightAngles


class CoreV1(Core):
    """The core module of a modular robot."""

    BOUNDING_BOX = Vector3([0.089, 0.089, 0.0603])
    MASS = 0.250
    CHILD_OFFSET = 0.089 / 2.0
    color = Color(255, 50, 50, 255)
    num_children = 4

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        """
        super().__init__(4, rotation)

    def build(
        self,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
        robot: Actor,
        dof_ids: list[int],
    ) -> None:
        """
        Build a v1 core onto the Robot.

        :param body: The rigid body.
        :param name_prefix: The name prefix.
        :param attachment_point: The attachment point.
        :param orientation: The modules Orientation.
        :param robot: The actor.
        :param dof_ids: dof ids.
        """
        # attachment position is always at center of core
        position = attachment_point

        body.collisions.append(
            Collision(
                name=f"{name_prefix}_core_collision",
                position=position,
                orientation=orientation,
                mass=self.MASS,
                bounding_box=self.BOUNDING_BOX,
                color=self.color,
            )
        )

        for name_suffix, child_index, angle in [
            ("front", self.FRONT, 0.0),
            ("back", self.BACK, math.pi),
            ("left", self.LEFT, math.pi / 2.0),
            ("right", self.RIGHT, math.pi / 2.0 * 3),
        ]:
            child = self.children[child_index]
            if child is not None:
                rotation = (
                    orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([child.rotation, 0, 0])
                )
                child.build(
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([self.CHILD_OFFSET, 0.0, 0.0]),
                    rotation,
                    robot,
                    dof_ids,
                )
