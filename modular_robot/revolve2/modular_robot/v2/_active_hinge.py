from revolve2.modular_robot._active_hinge import ActiveHinge as AH
from revolve2.modular_robot._properties import Properties
from revolve2.modular_robot._right_angles import RightAngles
from revolve2.simulation.actor import Color


class ActiveHinge(AH):
    """
    An active hinge module for a modular robot.

    This is a rotary joint.
    """

    # angle range of servo
    # 60 degrees to each side
    RANGE = 1.047197551
    # max effort of servo
    # motor specs: 9.4 kgfcm at 4.8V or 11 kgfcm at 6.0V
    # about 9.6667 kgfcm at 5.0V, our operating voltage
    # 9.6667 * 9.807 / 100
    EFFORT = 0.948013269
    # max velocity of servo
    # motor specs: 0.17 s/60deg at 4.8V or 0.14 s/60deg at 6.0V
    # about 0.1652 s/60deg at 5.0V, our operating voltage
    # 1 / 0.1652 * 60 / 360 * 2pi
    VELOCITY = 6.338968228
    _attachment_position: int

    def __init__(
        self,
        rotation: float | RightAngles,
        attachment_position: int = 5,
        color: Color = Color(255, 255, 255, 255),
    ):
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        :param attachment_position: The modules attachment position.
        """
        self._attachment_position = attachment_position
        if isinstance(rotation, RightAngles):
            rotation_converted = rotation.value
        else:
            rotation_converted = rotation
        properties = Properties(
            attachment_position=attachment_position,
            rotation=rotation_converted,
            color=color,
        )
        super().__init__(
            self.RANGE,
            self.EFFORT,
            self.VELOCITY,
            properties,
        )

    @property
    def attachment_position(self) -> int:
        """
        Get the attachment position of Module.

        :returns: The position.
        """
        return self._attachment_position
