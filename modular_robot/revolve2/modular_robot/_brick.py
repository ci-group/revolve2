from ._module import Module
from ._right_angles import RightAngles
from revolve2.simulation.actor import Color


class Brick(Module):
    """A brick module for a modular robot."""

    FRONT = 0
    RIGHT = 1
    LEFT = 2

    def __init__(
        self, rotation: float | RightAngles, color: Color = Color(255, 50, 50)
    ):
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        """
        if isinstance(rotation, RightAngles):
            rotation_converted = rotation.value
        else:
            rotation_converted = rotation
        super().__init__(3, rotation_converted, color)

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
