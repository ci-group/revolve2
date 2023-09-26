from pyrr import Quaternion, Vector3
from revolve2.simulation.actor import Actor, Collision, Color, Joint, RigidBody

from .._common import ActiveHinge
from .._right_angles import RightAngles


class ActiveHingeV1(ActiveHinge):
    """
    An active hinge module for a modular robot.

    This is a rotary joint.
    """

    RANGE = 1.047197551  # 60 degrees to each side.
    EFFORT = 0.948013269  # motor specs: 9.4 kgfcm at 4.8V or 11 kgfcm at 6.0V -> at 5.0V: 9.6667 * 9.807 / 100
    VELOCITY = 6.338968228  # motor specs: 0.17 s/60deg at 4.8V or 0.14 s/60deg at 6.0V -> at 5.0V: 1 / 0.1652 * 60 / 360 * 2pi
    FRAME_BOUNDING_BOX = Vector3([0.018, 0.053, 0.0165891])
    FRAME_OFFSET = 0.04525
    SERVO1_BOUNDING_BOX = Vector3([0.0583, 0.0512, 0.020])
    SERVO2_BOUNDING_BOX = Vector3([0.002, 0.053, 0.053])
    FRAME_MASS = 0.011
    SERVO1_MASS = 0.058
    SERVO2_MASS = 0.02
    SERVO_OFFSET = 0.0299
    JOINT_OFFSET = 0.0119
    _STATIC_FRICTION = 1.0
    _DYNAMIC_FRICTION = 1.0
    color = Color(255, 255, 255, 255)
    num_children = 1

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        """
        super().__init__(1, rotation)

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
        Build a v1 hinge onto the Robot.

        :param body: The rigid body.
        :param name_prefix: The name prefix.
        :param attachment_point: The attachment point.
        :param orientation: The modules Orientation.
        :param robot: The actor.
        :param dof_ids: dof ids.
        """
        SERVO_BBOX2_POSITION = Vector3(
            [
                self.SERVO1_BOUNDING_BOX[0] / 2.0 + self.SERVO2_BOUNDING_BOX[0] / 2.0,
                0.0,
                0.0,
            ]
        )

        ATTACHMENT_OFFSET = (
            self.SERVO1_BOUNDING_BOX[0] / 2.0 + self.SERVO2_BOUNDING_BOX[0]
        )

        frame_position = attachment_point + orientation * Vector3(
            [self.FRAME_OFFSET / 2.0, 0.0, 0.0]
        )
        frame_position_real = attachment_point + orientation * Vector3(
            [self.FRAME_BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )
        servo_body_position = body.position + body.orientation * (
            frame_position + orientation * Vector3([self.SERVO_OFFSET, 0.0, 0.0])
        )
        servo_body_orientation = body.orientation * orientation
        joint_position = body.position + body.orientation * (
            frame_position + orientation * Vector3([self.JOINT_OFFSET, 0.0, 0.0])
        )
        joint_orientation = body.orientation * orientation

        body.collisions.append(
            Collision(
                name=f"{name_prefix}_activehingeframe_collision",
                position=frame_position_real,
                orientation=orientation,
                mass=self.FRAME_MASS,
                bounding_box=self.FRAME_BOUNDING_BOX,
                color=self.color,
            )
        )

        next_body = RigidBody(
            f"{name_prefix}_activehinge",
            servo_body_position,
            servo_body_orientation,
            self._STATIC_FRICTION,
            self._DYNAMIC_FRICTION,
        )
        robot.bodies.append(next_body)
        robot.joints.append(
            Joint(
                f"{name_prefix}_activehinge",
                body,
                next_body,
                joint_position,
                joint_orientation,
                Vector3([0.0, 1.0, 0.0]),
                range=self.RANGE,
                effort=self.EFFORT,
                velocity=self.VELOCITY,
            )
        )
        dof_ids.append(self.id)

        next_body.collisions.append(
            Collision(
                name=f"{name_prefix}_activehingemotor_collision1",
                position=Vector3(),
                orientation=Quaternion(),
                mass=self.SERVO1_MASS,
                bounding_box=self.SERVO1_BOUNDING_BOX,
                color=self.color,
            )
        )
        next_body.collisions.append(
            Collision(
                name=f"{name_prefix}_activehingemotor_collision2",
                position=SERVO_BBOX2_POSITION,
                orientation=Quaternion(),
                mass=self.SERVO2_MASS,
                bounding_box=self.SERVO2_BOUNDING_BOX,
                color=self.color,
            )
        )

        child = self.children[self.ATTACHMENT]
        if child is not None:
            rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])
            child.build(
                next_body,
                f"{name_prefix}_attachment",
                rotation * Vector3([ATTACHMENT_OFFSET, 0.0, 0.0]),
                rotation,
                robot,
                dof_ids,
            )
