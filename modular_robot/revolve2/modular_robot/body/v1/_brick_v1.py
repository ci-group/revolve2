import math

from pyrr import Quaternion, Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._right_angles import RightAngles
from ..base import Brick


class BrickV1(Brick):
    """A brick module for a modular robot."""

    _COLOR = Color(50, 50, 255, 255)

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules' rotation.
        """
        child_offset = Vector3([0.06288625 / 2.0, 0.0, 0.0])
        attachment_points = {
            self.FRONT: AttachmentPoint(
                rotation=Quaternion.from_eulers([0.0, 0.0, 0.0]), offset=child_offset
            ),
            self.LEFT: AttachmentPoint(
                rotation=Quaternion.from_eulers([0.0, 0.0, math.pi / 2.0]),
                offset=child_offset,
            ),
            self.RIGHT: AttachmentPoint(
                rotation=Quaternion.from_eulers([0.0, 0.0, math.pi / 2.0 * 3]),
                offset=child_offset,
            ),
        }

        super().__init__(
            rotation=rotation,
            color=self._COLOR,
            bounding_box=Vector3([0.06288625, 0.06288625, 0.0603]),
            mass=0.030,
            attachment_points=attachment_points,
        )
