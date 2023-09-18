from revolve2.modular_robot._core import Core as C
from revolve2.modular_robot._properties import Properties
from revolve2.modular_robot._right_angles import RightAngles
from revolve2.simulation.actor import Color


class Core(C):
    """The core module of a modular robot."""

    def __init__(
        self, rotation: float | RightAngles, color: Color = Color(255, 50, 50, 255)
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
