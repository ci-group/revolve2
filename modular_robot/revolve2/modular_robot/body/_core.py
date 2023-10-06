from ._color import Color
from ._module import Module
from ._right_angles import RightAngles


class Core(Module):
    """The core module of a modular robot."""

    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    _last_unique_id: int
    """
    This is the last assigned id to a module.
    
    See `_get_new_module_id` on what this is used for.
    """

    def __init__(
        self, rotation: float | RightAngles, color: Color = Color(255, 50, 50, 255)
    ):
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        """
        if isinstance(rotation, RightAngles):
            rotation_converted = rotation.value
        else:
            rotation_converted = rotation
        super().__init__(4, rotation_converted, color)
        self._id = 0
        self._parent = None
        self._parent_child_index = None
        self._last_unique_id = self._id

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

    def _get_new_module_id(self) -> int:
        """
        Get a new module id, unique within the parent body.

        This is the core, which has been tasked to manage unique ids.
        As the core is always the root of the body, there is never more than one core.

        :returns: The unique id.
        """
        self._last_unique_id += 1
        return self._last_unique_id
