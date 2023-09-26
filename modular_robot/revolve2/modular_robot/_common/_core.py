from .._module import Module
from .._right_angles import RightAngles

class Core(Module):
    """A Core."""

    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    def __init__(self, num_children: int, rotation: float | RightAngles):
        """
        Initialize this object.

        :param num_children: The number of children.
        :param rotation: The Modules rotation.
        """
        super().__init__(num_children, rotation)

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
        self.children[self.FRONT] = module

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
        self.children[self.RIGHT] = module

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
        self.children[self.BACK] = module

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
        self.children[self.LEFT] = module