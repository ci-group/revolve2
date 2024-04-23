import math

from pyrr import Quaternion, Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._module import Module
from .._right_angles import RightAngles
from ..sensors import Sensor


class Brick(Module):
    """A Brick Module."""

    FRONT = 0
    RIGHT = 1
    LEFT = 2

    _mass: float
    _bounding_box: Vector3

    def __init__(
        self,
        rotation: float | RightAngles,
        mass: float,
        bounding_box: Vector3,
        child_offset: float,
        sensors: list[Sensor],
    ):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half-length from origin.
        :param child_offset: The offset of the child for each attachment point.
        :param sensors: The sensors associated with this module.
        """
        attachment_points = {
            self.FRONT: AttachmentPoint(
                offset=Vector3([child_offset, 0.0, 0.0]),
                orientation=Quaternion.from_eulers([0.0, 0.0, 0.0]),
            ),
            self.LEFT: AttachmentPoint(
                offset=Vector3([child_offset, 0.0, 0.0]),
                orientation=Quaternion.from_eulers([0.0, 0.0, math.pi / 2.0]),
            ),
            self.RIGHT: AttachmentPoint(
                offset=Vector3([child_offset, 0.0, 0.0]),
                orientation=Quaternion.from_eulers([0.0, 0.0, math.pi / 2.0 * 3]),
            ),
        }
        self._mass = mass
        self._bounding_box = bounding_box

        """
        The base module only has orientation as its parameter since not all modules are square.

        Here we covert the angle of the module to its orientation in space.
        """
        orientation = Quaternion.from_eulers([rotation if isinstance(rotation, float) else rotation.value, 0, 0])
        super().__init__(orientation, Color(50, 50, 255, 255), attachment_points, sensors)

    @property
    def front(self) -> Module | None:
        """
        Get the front module of the brick.

        :returns: The attachment points module.
        """
        return self._children.get(self.FRONT)

    @front.setter
    def front(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self.set_child(module, self.FRONT)

    @property
    def right(self) -> Module | None:
        """
        Get right module of the brick.

        :returns: The attachment points module.
        """
        return self._children.get(self.RIGHT)

    @right.setter
    def right(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self.set_child(module, self.RIGHT)

    @property
    def left(self) -> Module | None:
        """
        Get the left module of the brick.

        :returns: The attachment points module.
        """
        return self._children.get(self.LEFT)

    @left.setter
    def left(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
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

        Sizes are total length, not half-length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (in m).
        """
        return self._bounding_box
