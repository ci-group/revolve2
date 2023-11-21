import math

from pyrr import Quaternion, Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._module import Module
from .._right_angles import RightAngles


class Core(Module):
    """The core module of a modular robot."""

    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    _bounding_box: Vector3
    _mass: float
    """
    This is the last assigned id to a module.
    See `_get_new_module_id` on what this is used for.
    """

    def __init__(
        self,
        rotation: float | RightAngles,
        mass: float,
        bounding_box: Vector3,
        child_offset: Vector3,
        color: Color = Color(255, 50, 50, 255),
    ):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param child_offset: The offset for children to be attached to the attachment points.
        :param color: The Modules color.
        """
        self._mass = mass
        self._bounding_box = bounding_box

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

        super().__init__(rotation, color, attachment_points)
        self._parent = None
        self._parent_child_index = None

    @property
    def mass(self) -> float:
        """
        Get the mass of the Core (in kg).

        :return: The value.
        """
        return self._mass

    @property
    def bounding_box(self) -> Vector3:
        """
        Get the bounding box.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (m).
        """
        return self._bounding_box
