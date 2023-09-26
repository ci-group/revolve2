import math

from pyrr import Quaternion, Vector3
from revolve2.simulation.actor import Actor, Collision, Color, RigidBody

from .._brick import Brick
from .._right_angles import RightAngles


class BrickV1(Brick):
    """A brick module for a modular robot."""

    BOUNDING_BOX = Vector3([0.06288625, 0.06288625, 0.0603])
    MASS = 0.030
    CHILD_OFFSET = 0.06288625 / 2.0
    color = Color(50, 50, 255, 255)
    num_children = 3

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        """
        super().__init__(3, rotation)

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
        Build a v1 brick onto the Robot.

        :param body: The rigid body.
        :param name_prefix: The name prefix.
        :param attachment_point: The attachment point.
        :param orientation: The modules Orientation.
        :param robot: The actor.
        :param dof_ids: dof ids.
        """
        position = attachment_point + orientation * Vector3(
            [self.BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )

        body.collisions.append(
            Collision(
                name=f"{name_prefix}_brick_collision",
                position=position,
                orientation=orientation,
                mass=self.MASS,
                bounding_box=self.BOUNDING_BOX,
                color=self.color,
            )
        )

        for name_suffix, child_index, angle in [
            ("front", self.FRONT, 0.0),
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
