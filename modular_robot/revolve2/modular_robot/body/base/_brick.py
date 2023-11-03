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
        Get the module attached to the front of the brick.

        :returns: The attached module.
        """
        return self.children[self.FRONT]

    @front.setter
    def front(self, module: Module) -> None:
        """
        Set the module attached to the front of the brick.

        :param module: The module to attach.
        """
        self.set_child(module, self.FRONT)

    @property
    def right(self) -> Module | None:
        """
        Get the module attached to the right of the brick.

        :returns: The attached module.
        """
        return self.children[self.RIGHT]

    @right.setter
    def right(self, module: Module) -> None:
        """
        Set the module attached to the right of the brick.

        :param module: The module to attach.
        """
        self.set_child(module, self.RIGHT)

    @property
    def left(self) -> Module | None:
        """
        Get the module attached to the left of the brick.

        :returns: The attached module.
        """
        return self.children[self.LEFT]

    @left.setter
    def left(self, module: Module) -> None:
        """
        Set the module attached to the left of the brick.

        :param module: The module to attach.
        """
        self.set_child(module, self.LEFT)

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
