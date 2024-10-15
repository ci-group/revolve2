from pyrr import Vector3

from .._right_angles import RightAngles
from ..base import Brick


class BrickV2Large(Brick):
    """A brick module for a modular robot."""

    def __init__(self, rotation: float | RightAngles, offset=None):
        """
        Initialize this object.

        :param rotation: The modules' rotation.
        """
        super().__init__(
            rotation=rotation,
            bounding_box=Vector3([0.15, 0.075, 0.075]),
            mass=(42.65 + (0.15-0.075) * 0.44531428571) / 1000, # Might need to update later
            child_offset=offset / 2.0 if offset else 0.15 / 2.0,
            sensors=[],
        )
