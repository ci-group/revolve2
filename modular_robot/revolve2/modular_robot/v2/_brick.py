from revolve2.modular_robot._brick import Brick as B
from revolve2.modular_robot._properties import Properties
from revolve2.modular_robot._right_angles import RightAngles
from revolve2.simulation.actor import Color


class Brick(B):
    """A brick module for a modular robot."""

    FRONT = 0
    RIGHT = 1
    LEFT = 2

    _attachment_position: int

    def __init__(
        self,
        rotation: float | RightAngles,
        attachment_position: int = 5,
        color: Color = Color(50, 50, 255, 255),
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
        super().__init__(properties)

    @property
    def attachment_position(self) -> int:
        """
        Get the attachment position of Module.

        :returns: The position.
        """
        return self._attachment_position
