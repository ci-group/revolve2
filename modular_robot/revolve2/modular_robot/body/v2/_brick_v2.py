from pyrr import Vector3

from .._color import Color
from .._right_angles import RightAngles
from ..base import Brick


class BrickV2(Brick):
    """A brick module for a modular robot."""

    _COLOR = Color(50, 50, 255, 255)

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        """
        super().__init__(
            num_children=3,
            rotation=rotation,
            color=self._COLOR,
            bounding_box=Vector3([0.06288625, 0.06288625, 0.0603]),
            mass=0.06043,
            child_offset=0.06288625 / 2.0,
        )
