from revolve2.modular_robot._module import Module
from revolve2.modular_robot._right_angles import RightAngles


class ActiveHinge(Module):
    """
    An active hinge module for a modular robot.

    This is a rotary joint.
    """

    ATTACHMENT = 0

    def __init__(self, rotation: float | RightAngles, attachment_position: int = 5):
        """
        Initialize this object.

        :param rotation: The Modules rotation.
        :param attachment_position: The Modules attachment position.
        """
        super().__init__(1, rotation, attachment_position)

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
