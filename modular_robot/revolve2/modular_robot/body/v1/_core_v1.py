import math

from pyrr import Quaternion, Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._module import Module
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
        child_offset = Vector3([0.089 / 2.0, 0.0, 0.0])
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
            bounding_box=Vector3([0.089, 0.089, 0.0603]),
            mass=0.250,
            attachment_points=attachment_points,
        )

    @property
    def front(self) -> Module | None:
        """
        Get the front attachment point of the core.

        :returns: The attachment points module.
        """
        return self._attachment_points[self.FRONT].module

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
        Get the right attachment point of the core.

        :returns: The attachment points module.
        """
        return self._attachment_points[self.RIGHT].module

    @right.setter
    def right(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self._attachment_points[self.RIGHT].module = module

    @property
    def back(self) -> Module | None:
        """
        Get the back attachment point of the core.

        :returns: The attachment points module.
        """
        return self._attachment_points[self.BACK].module

    @back.setter
    def back(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self._attachment_points[self.BACK].module = module

    @property
    def left(self) -> Module | None:
        """
        Get the left attachment point of the core.

        :returns: The attachment points module.
        """
        return self._attachment_points[self.LEFT].module

    @left.setter
    def left(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self._attachment_points[self.LEFT].module = module
