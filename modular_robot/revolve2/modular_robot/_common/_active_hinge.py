from .._module import Module
from .._right_angles import RightAngles

class ActiveHinge(Module):
    """A Active Hinge."""

    ATTACHMENT = 0

    def __init__(self, num_children: int, rotation: float | RightAngles):
        """
        Initialize this object.

        :param num_children: The number of children.
        :param rotation: The Modules rotation.
        """
        super().__init__(num_children, rotation)

    @property
    def attachment(self) -> Module | None:
        """
        Get the module attached to this hinge.

        :returns: The attached module.
        """
        return self.children[self.ATTACHMENT]

    @attachment.setter
    def attachment(self, module: Module) -> None:
        """
        Set the module attached to this hinge.

        :param module: The module to attach.
        """
        self.children[self.ATTACHMENT] = module
