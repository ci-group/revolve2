from __future__ import annotations

from pyrr import Vector3

from .._color import Color
from .._module import Module
from .._right_angles import RightAngles


class ActiveHinge(Module):
    """An Active Hinge."""

    ATTACHMENT = 0

    _range: float
    _effort: float
    _velocity: float

    _servo1_bounding_box: Vector3
    _servo2_bounding_box: Vector3
    _frame_bounding_box: Vector3
    _frame_offset: float
    _servo_offset: float
    _frame_mass: float
    _servo1_mass: float
    _servo2_mass: float
    _joint_offset: float
    _static_friction: float
    _dynamic_friction: float

    def __init__(
        self,
        num_children: int,
        rotation: float | RightAngles,
        color: Color,
        servo1_bounding_box: Vector3,
        servo2_bounding_box: Vector3,
        frame_bounding_box: Vector3,
        frame_offset: float,
        servo_offset: float,
        frame_mass: float,
        servo1_mass: float,
        servo2_mass: float,
        joint_offset: float,
        static_friction: float,
        dynamic_friction: float,
        range: float,
        effort: float,
        velocity: float,
    ):
        """
        Initialize this object.

        :param num_children: The number of children.
        :param rotation: The Modules rotation.
        :param color: The Modules color.
        :param servo1_bounding_box: The bounding box of servo 1. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param servo2_bounding_box: The bounding box of servo 2. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param frame_bounding_box: The bounding box of the frame. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param frame_offset: The offset of the frame (m).
        :param servo_offset: The servo offset (m).
        :param frame_mass: The frame mass (kg).
        :param servo1_mass: Servo 1 mass (kg).
        :param servo2_mass: Servo 2 mass (kg).
        :param joint_offset: The joints offset (m).
        :param static_friction: The static friction of servos.
        :param dynamic_friction: The dynamic friction of servos.
        :param range: The range of motion for servos (radiants).
        :param effort: The effort of servos (kgfcm/10).
        :param velocity: The velocity of servos (1/sec/60deg*1/3pi).
        """
        self._static_friction = static_friction
        self._dynamic_friction = dynamic_friction
        self._joint_offset = joint_offset
        self._frame_offset = frame_offset
        self._servo_offset = servo_offset
        self._effort = effort
        self._velocity = velocity
        self._range = range
        self._frame_mass = frame_mass
        self._servo1_mass = servo1_mass
        self._servo2_mass = servo2_mass
        self._frame_bounding_box = frame_bounding_box
        self._servo1_bounding_box = servo1_bounding_box
        self._servo2_bounding_box = servo2_bounding_box
        super().__init__(num_children, rotation, color)

    @property
    def attachment(self) -> Module | None:
        """
        Get the module attached to this hinge.

        :returns: The attached module.
        """
        return self.children[self.ATTACHMENT]

    @attachment.setter
    def attachment(self, module: Module) -> None:
        """
        Set the module attached to this hinge.

        :param module: The module to attach.
        """
        self.set_child(module, self.ATTACHMENT)

    @property
    def static_friction(self) -> float:
        """
        Get the static friction.

        :return: The value.
        """
        return self._static_friction

    @property
    def dynamic_friction(self) -> float:
        """
        Get the dynamic friction.

        :return: The value.
        """
        return self._dynamic_friction

    @property
    def range(self) -> float:
        """
        Get the range of the servo.

        :return: The value.
        """
        return self._range

    @property
    def effort(self) -> float:
        """
        Get the effort of the servo.

        :return: The value.
        """
        return self._effort

    @property
    def velocity(self) -> float:
        """
        Get the velocity of the servo.

        :return: The value.
        """
        return self._velocity

    @property
    def servo1_bounding_box(self) -> Vector3:
        """
        Get the bounding box of the first servo part.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (m).
        """
        return self._servo1_bounding_box

    @property
    def servo2_bounding_box(self) -> Vector3:
        """
        Get the bounding box of the second servo part.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (m).
        """
        return self._servo2_bounding_box

    @property
    def frame_bounding_box(self) -> Vector3:
        """
        Get the bounding box of the frame.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (m).
        """
        return self._frame_bounding_box

    @property
    def frame_offset(self) -> float:
        """
        Get the offset of the frame (in m).

        :return: The value.
        """
        return self._frame_offset

    @property
    def servo_offset(self) -> float:
        """
        Get the servo offset (in m).

        :return: The value.
        """
        return self._servo_offset

    @property
    def frame_mass(self) -> float:
        """
        Get the frame mass (in kg).

        :return: The value.
        """
        return self._frame_mass

    @property
    def servo1_mass(self) -> float:
        """
        Get the mass of the first servo part (in kg).

        :return: The value.
        """
        return self._servo1_mass

    @property
    def servo2_mass(self) -> float:
        """
        Get the mass of the second servo part (in kg).

        :return: The value.
        """
        return self._servo2_mass

    @property
    def joint_offset(self) -> float:
        """
        Get the joint offset (in m).

        :return: The value.
        """
        return self._joint_offset
