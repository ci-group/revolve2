import math

from pyrr import Quaternion, Vector3
from revolve2.simulation.actor import Actor, Collision, Color, RigidBody

from .._core import Core
from .._right_angles import RightAngles


class CoreV2(Core):
    """The core module of a modular robot."""

    BOUNDING_BOX = Vector3([0.15, 0.15, 0.15])
    MASS = 0.250
    CHILD_OFFSET = 0.15 / 2.0
    color = Color(255, 50, 50, 255)
    num_children = 4
    HORIZONTAL_OFFSET = 0.029
    VERTICAL_OFFSET = 0.032
    attachment_positions: list[int]  # TODO: make this into meta-class

    def __init__(self, rotation: float | RightAngles, attachment_positions: list[int]):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        :param attachment_positions: The attachment positions of children.
        """
        self.attachment_positions = attachment_positions
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
        Build a v2 core onto the Robot.

        :param body: The rigid body.
        :param name_prefix: The name prefix.
        :param attachment_point: The attachment point.
        :param orientation: The modules Orientation.
        :param robot: The actor.
        :param dof_ids: dof ids.
        """

        def __get_attachment_offset(attachment_position: int, angle: float) -> Vector3:
            """
            Calculate offset.

            :param attachment_position: index position of attachment on core.
            :param angle: angle of core side for determining offset axis / direction.
            :returns: The offset Vector.
            """
            HO, VO = self.HORIZONTAL_OFFSET, self.VERTICAL_OFFSET
            h_cond, v_cond = attachment_position % 3, attachment_position / 3

            h_offset, v_offset = 0.0, 0.0
            if h_cond == 1:
                h_offset = -HO
            elif h_cond == 0:
                h_offset = HO
            h_offset = -h_offset if angle >= math.pi else h_offset
            if v_cond <= 1:
                v_offset = VO
            elif v_cond > 2:
                v_offset = -VO

            offset = (
                Vector3([0.0, h_offset, v_offset])
                if angle % math.pi == 0
                else Vector3([-h_offset, 0.0, v_offset])
            )
            return offset

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

                attachment_offset = (
                    __get_attachment_offset(
                        self.attachment_positions[child_index], angle
                    )
                    if self.attachment_positions[child_index] > 0
                    else Vector3([0.0, 0.0, 0.0])
                )

                child.build(
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position
                    + rotation * Vector3([self.CHILD_OFFSET, 0.0, 0.0])
                    + attachment_offset,
                    rotation,
                    robot,
                    dof_ids,
                )
