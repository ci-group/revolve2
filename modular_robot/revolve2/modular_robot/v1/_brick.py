from revolve2.modular_robot._brick import Brick as B
from revolve2.modular_robot._properties import Properties
from revolve2.modular_robot._right_angles import RightAngles
from revolve2.simulation.actor import Color


class Brick(B):
    """A brick module for a modular robot."""

    FRONT = 0
    RIGHT = 1
    LEFT = 2

    def __init__(
        self, rotation: float | RightAngles, color: Color = Color(50, 50, 255, 255)
    ):
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        """
        if isinstance(rotation, RightAngles):
            rotation_converted = rotation.value
        else:
            rotation_converted = rotation
        properties = Properties(color=color, rotation=rotation_converted)
        super().__init__(properties)
