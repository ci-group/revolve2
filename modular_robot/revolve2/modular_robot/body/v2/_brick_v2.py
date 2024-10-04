from pyrr import Vector3

from .._right_angles import RightAngles
from ..base import Brick


class BrickV2(Brick):
    """A brick module for a modular robot."""

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules' rotation.
        """
        w, h, d = 0.075, 0.075, 0.075
        super().__init__(
            rotation=rotation,
            bounding_box=Vector3([w, h, d]),
            mass=0.06043,
            child_offset=d / 2.0,
            sensors=[],
        )
