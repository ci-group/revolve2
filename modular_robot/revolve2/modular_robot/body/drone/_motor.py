from pyrr import Quaternion, Vector3

from ..base import Motor


class MotorImpl(Motor):
    """An motor module for a modular robot."""

    def __init__(
        self, position: Vector3, orientation: Quaternion, clockwise_rotation: bool
    ) -> None:
        """
        Initialize this object.

        :param position: The Modules position with respect to the core.
        :param orientation: The Modules orientation with respect to the core.
        :param clockwise_rotation: The rotation direction of the rotor. True is clockwise.
        """
        super().__init__(
            position=position,
            orientation=orientation,
            rotor_size=[0.05, 0.018],
            frame_size=[0.018, 0.018],
            mass=0.025,
            control_range=[0, 100],
            clockwise_rotation=clockwise_rotation,
        )
