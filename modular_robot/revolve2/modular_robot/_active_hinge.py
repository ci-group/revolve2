from revolve2.simulation.actor import Color

from ._module import Module
from ._right_angles import RightAngles


class ActiveHinge(Module):
    """
    An active hinge module for a modular robot.

    This is a rotary joint.
    """

    ATTACHMENT = 0

    # angle range of servo
    # 60 degrees to each side
    RANGE = 1.047197551
    # max effort of servo
    # motor specs: 9.4 kgfcm at 4.8V or 11 kgfcm at 6.0V
    # about 9.6667 kgfcm at 5.0V, our operating voltage
    # 9.6667 * 9.807 / 100
    EFFORT = 0.948013269
    # max velocity of servo
    # motor specs: 0.17 s/60deg at 4.8V or 0.14 s/60deg at 6.0V
    # about 0.1652 s/60deg at 5.0V, our operating voltage
    # 1 / 0.1652 * 60 / 360 * 2pi
    VELOCITY = 6.338968228

    def __init__(
        self, rotation: float | RightAngles, color: Color = Color(255, 255, 255, 255)
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
        super().__init__(1, rotation_converted, color)

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
