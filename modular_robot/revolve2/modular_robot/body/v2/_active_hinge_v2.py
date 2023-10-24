from pyrr import Vector3

from .._color import Color
from .._right_angles import RightAngles
from ..base import ActiveHinge


class ActiveHingeV2(ActiveHinge):
    """
    An active hinge module for a modular robot.

    This is a rotary joint.
    """

    _COLOR = Color(255, 255, 255, 255)

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        """
        super().__init__(
            num_children=1,
            rotation=rotation,
            color=self._COLOR,
            range=1.047197551,
            effort=0.948013269,
            velocity=6.338968228,
            frame_bounding_box=Vector3([0.018, 0.053, 0.0165891]),
            frame_offset=0.04525,
            servo1_bounding_box=Vector3([0.0583, 0.0512, 0.020]),
            servo2_bounding_box=Vector3([0.002, 0.053, 0.053]),
            frame_mass=0.01632,
            servo1_mass=0.058,
            servo2_mass=0.025,
            servo_offset=0.0299,
            joint_offset=0.0119,
            static_friction=1.0,
            dynamic_friction=1.0,
            armature=0.002,
            pid_gain_p=5.0,
            pid_gain_d=0.05,
        )
