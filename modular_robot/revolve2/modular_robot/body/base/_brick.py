from pyrr import Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._module import Module
from .._right_angles import RightAngles


class Brick(Module):
    """A Brick."""

    FRONT = 0
    RIGHT = 1
    LEFT = 2

    _mass: float
    _bounding_box: Vector3

    def __init__(
        self,
        rotation: float | RightAngles,
        color: Color,
        mass: float,
        bounding_box: Vector3,
        attachment_points: dict[int, AttachmentPoint],
    ):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        :param color: The Modules color.
        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param attachment_points: The attachment points available on a module.
        """
        self._mass = mass
        self._bounding_box = bounding_box
        super().__init__(rotation, color, attachment_points)

    @property
    def front(self) -> Module | None:
        """
        Get the front attachment point of the brick.

        :returns: The attachment points module.
        """
        return self._attachment_points.get(self.FRONT).module

    @front.setter
    def front(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self._attachment_points[self.FRONT].module = module

    @property
    def right(self) -> Module | None:
        """
        Get right attachment point of the brick.

        :returns: The attachment points module.
        """
        return self._attachment_points.get(self.FRONT).module

    @right.setter
    def right(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self._attachment_points[self.RIGHT].module = module

    @property
    def left(self) -> Module | None:
        """
        Get the left attachment point of the brick.

        :returns: The attachment points module.
        """
        return self._attachment_points.get(self.FRONT).module

    @left.setter
    def left(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self._attachment_points[self.LEFT].module = module

    @property
    def mass(self) -> float:
        """
        Get the mass of the brick (in kg).

        :return: The value.
        """
        return self._mass

    @property
    def bounding_box(self) -> Vector3:
        """
        Get the bounding box size.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (in m).
        """
        return self._bounding_box
