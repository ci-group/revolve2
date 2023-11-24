from pyrr import Vector3

from .._module import Module
from .._right_angles import RightAngles
from ..base import Core


class CoreV1(Core):
    """The core module of a modular robot."""

    def __init__(self, rotation: float | RightAngles):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        """
        child_offset = Vector3([0.089 / 2.0, 0.0, 0.0])

        super().__init__(
            rotation=rotation,
            bounding_box=Vector3([0.089, 0.089, 0.0603]),
            mass=0.250,
            child_offset=child_offset,
        )

    @property
    def front(self) -> Module | None:
        """
        Get the front attachment point of the core.

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
        Get the right attachment point of the core.

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
    def back(self) -> Module | None:
        """
        Get the back attachment point of the core.

        :returns: The attachment points module.
        """
        return self._children.get(self.BACK)

    @back.setter
    def back(self, module: Module) -> None:
        """
        Set a module onto the attachment point.

        :param module: The Module.
        """
        self.set_child(module, self.BACK)

    @property
    def left(self) -> Module | None:
        """
        Get the left attachment point of the core.

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
