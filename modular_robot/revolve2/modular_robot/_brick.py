from revolve2.modular_robot._module import Module
from revolve2.modular_robot._properties import Properties


class Brick(Module):
    """A brick module for a modular robot."""

    FRONT = 0
    RIGHT = 1
    LEFT = 2

    def __init__(self, properties: Properties):
        """
        Initialize this object.

        :param properties: The modules properties.
        """
        properties.num_children = 3
        super().__init__(properties)

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
        self.children[self.FRONT] = module

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
        self.children[self.RIGHT] = module

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
        self.children[self.LEFT] = module
