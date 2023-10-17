from pyrr import Vector3

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
    _child_offset: float
    """
    This is the last assigned id to a module.
    See `_get_new_module_id` on what this is used for.
    """

    def __init__(
        self,
        num_children: int,
        rotation: float | RightAngles,
        color: Color,
        mass: float,
        bounding_box: Vector3,
        child_offset: float,
    ):
        """
        Initialize this object.

        :param num_children: The number of children.
        :param rotation: The Modules rotation.
        :param color: The Modules color.
        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        :param child_offset: The child offset (in m).
        """
        self._mass = mass
        self._child_offset = child_offset
        self._bounding_box = bounding_box
        super().__init__(num_children, rotation, color)
        self._parent = None
        self._parent_child_index = None

    @property
    def front(self) -> Module | None:
        """
        Get the module attached to the front of the core.

        :returns: The attached module.
        """
        return self.children[self.FRONT]

    @front.setter
    def front(self, module: Module) -> None:
        """
        Set the module attached to the front of the core.

        :param module: The module to attach.
        """
        self.set_child(module, self.FRONT)

    @property
    def right(self) -> Module | None:
        """
        Get the module attached to the right of the core.

        :returns: The attached module.
        """
        return self.children[self.RIGHT]

    @right.setter
    def right(self, module: Module) -> None:
        """
        Set the module attached to the right of the core.

        :param module: The module to attach.
        """
        self.set_child(module, self.RIGHT)

    @property
    def back(self) -> Module | None:
        """
        Get the module attached to the back of the core.

        :returns: The attached module.
        """
        return self.children[self.BACK]

    @back.setter
    def back(self, module: Module) -> None:
        """
        Set the module attached to the back of the core.

        :param module: The module to attach.
        """
        self.set_child(module, self.BACK)

    @property
    def left(self) -> Module | None:
        """
        Get the module attached to the left of the core.

        :returns: The attached module.
        """
        return self.children[self.LEFT]

    @left.setter
    def left(self, module: Module) -> None:
        """
        Set the module attached to the left of the core.

        :param module: The module to attach.
        """
        self.set_child(module, self.LEFT)

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

    @property
    def child_offset(self) -> float:
        """
        Get the child offset (in m).

        :return: The value.
        """
        return self._child_offset
