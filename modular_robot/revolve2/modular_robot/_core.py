from revolve2.simulation.actor import Color

from ._module import Module
from ._directions import Directions, RightAngles


class Core(Module):
    """The core module of a modular robot."""

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

    @property
    def front(self) -> Module | None:
        """
        Get the module attached to the front of the core.

        :returns: The attached module.
        """
        return self.get_child(Directions.FRONT)

    @front.setter
    def front(self, module: Module) -> None:
        """
        Set the module attached to the front of the core.

        :param module: The module to attach.
        """
        self.set_child(module, Directions.FRONT)

    @property
    def right(self) -> Module | None:
        """
        Get the module attached to the right of the core.

        :returns: The attached module.
        """
        return self.get_child(Directions.RIGHT)

    @right.setter
    def right(self, module: Module) -> None:
        """
        Set the module attached to the right of the core.

        :param module: The module to attach.
        """
        self.set_child(module, Directions.RIGHT)

    @property
    def back(self) -> Module | None:
        """
        Get the module attached to the back of the core.

        :returns: The attached module.
        """
        return self.get_child(Directions.BACK)

    @back.setter
    def back(self, module: Module) -> None:
        """
        Set the module attached to the back of the core.

        :param module: The module to attach.
        """
        self.set_child(module, Directions.BACK)

    @property
    def left(self) -> Module | None:
        """
        Get the module attached to the left of the core.

        :returns: The attached module.
        """
        return self.get_child(Directions.LEFT)

    @left.setter
    def left(self, module: Module) -> None:
        """
        Set the module attached to the left of the core.

        :param module: The module to attach.
        """
        self.set_child(module, Directions.LEFT)
