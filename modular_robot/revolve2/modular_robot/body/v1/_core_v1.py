from pyrr import Vector3

from .._color import Color
from .._right_angles import RightAngles
from ..base import Core


class CoreV1(Core):
    """The core module of a modular robot."""

    _COLOR = Color(255, 50, 50, 255)

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        """
        super().__init__(
            num_children=4,
            rotation=rotation,
            color=self._COLOR,
            bounding_box=Vector3([0.089, 0.089, 0.0603]),
            mass=0.250,
            child_offset=0.089 / 2.0,
        )
