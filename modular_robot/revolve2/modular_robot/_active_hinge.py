from revolve2.modular_robot._module import Module
from revolve2.modular_robot._properties import Properties


class ActiveHinge(Module):
    """
    An active hinge module for a modular robot.

    This is a rotary joint.
    """

    ATTACHMENT = 0

    RANGE: float  # angle range of servo
    EFFORT: float  # max effort of servo
    VELOCITY: float  # max velocity of servo

    def __init__(
        self,
        range: float,
        effort: float,
        velocity: float,
        properties: Properties,
    ):
        """
        Initialize this object.

        :param effort: The Effort of the hinge.
        :param range: The Range of the hinge.
        :param velocity: The velocity of the hinge.
        :param properties: The properties of the module.
        """
        self.RANGE = range
        self.EFFORT = effort
        self.VELOCITY = velocity
        properties.num_children = 1
        super().__init__(properties)

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
