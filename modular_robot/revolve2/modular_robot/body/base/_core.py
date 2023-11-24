from pyrr import Vector3

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
        attachment_points: dict[int, AttachmentPoint],
    ):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param attachment_points: The attachment points for this core.
        """
        self._mass = mass
        self._bounding_box = bounding_box

        super().__init__(rotation, Color(255, 50, 50, 255), attachment_points)
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
