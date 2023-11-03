import math

from pyrr import Quaternion, Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._right_angles import RightAngles
from ..base import Core


class CoreV2(Core):
    """The core module of a modular robot."""

    _BATTERY_MASS = 0.39712  # in kg
    _FRAME_MASS = 1.0644  # in kg
    _COLOR = Color(255, 50, 50, 255)

    _horizontal_offset = 0.029  # The horizontal offset for attachment positions (in m).
    _vertical_offset = 0.032  # The vertical offset for attachment positions (in m).
    _attachment_positions: list[int]
    """
    Attachment positions provide the core with knowledge where child modules should be attached to. 
    They are indexed the same as children. (child[0] -> attachment_position[0]).
    The positions are arranged as such on each side of the core:
    
    ---------------------------------------------------
    |                 |            |                  |
    | 1 (TOP_LEFT)    | 2 (TOP)    | 3 (TOP_RIGHT)    |
    |                 |            |                  |
    | 4 (MIDDLE_LEFT) | 5 (MIDDLE) | 6 (MIDDLE_RIGHT) |
    |                 |            |                  |
    | 7 (BOTTOM_LEFT) | 8 (BOTTOM) | 9 (BOTTOM_RIGHT) |
    |                 |            |                  |
    ---------------------------------------------------
    """

    def __init__(
        self,
        rotation: float | RightAngles,
        attachment_positions: list[int],
        num_batteries: int = 1,
    ):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        :param attachment_positions: The attachment positions for child modules.
        :param num_batteries: The amount of batteries in the robot.
        """
        mass = (
            num_batteries * self._BATTERY_MASS + self._FRAME_MASS
        )  # adjust if multiple batteries are installed
        self._attachment_positions = attachment_positions

        child_offset = Vector3([0.15 / 2.0, 0.0, 0.0])
        attachment_points = {
            self.FRONT: AttachmentPoint(
                rotation=Quaternion.from_eulers([0.0, 0.0, 0.0]), offset=child_offset
            ),
            self.BACK: AttachmentPoint(
                rotation=Quaternion.from_eulers([0.0, 0.0, math.pi]),
                offset=child_offset,
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
            mass=mass,
            bounding_box=Vector3([0.15, 0.15, 0.15]),
            attachment_points=attachment_points,
        )

    @property
    def horizontal_offset(self) -> float:
        """
        Get the horizontal offset for attachment positions (in m).

        :return: The value.
        """
        return self._horizontal_offset

    @property
    def vertical_offset(self) -> float:
        """
        Get the vertical offset for attachment positions (in m).

        :return: The value.
        """
        return self._vertical_offset

    @property
    def attachment_positions(self) -> list[int]:
        """
        Get the populated attachment positions for each core side.

        :return: The value.
        """
        return self._attachment_positions
