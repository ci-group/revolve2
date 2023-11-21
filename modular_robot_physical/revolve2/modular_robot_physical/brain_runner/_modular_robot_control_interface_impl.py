from revolve2.modular_robot import ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge

from .._uuid_key import UUIDKey
from ..physical_interfaces import PhysicalInterface


class ModularRobotControlInterfaceImpl(ModularRobotControlInterface):
    """Implementation of ModularRobotControlInterface."""

    _hinge_mapping: dict[UUIDKey[ActiveHinge], int]
    _inverse_pin: dict[int, bool]
    _physical_interface: PhysicalInterface

    def __init__(
        self,
        hinge_mapping: dict[UUIDKey[ActiveHinge], int],
        inverse_pin: dict[int, bool],
        physical_interface: PhysicalInterface,
    ) -> None:
        """
        Initialize the PhysicalInterface.

        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        :param inverse_pin: If certain pins have to be reversed.
        :param physical_interface: Interface to the specific hardware.
        """
        self._inverse_pin = inverse_pin
        self._hinge_mapping = hinge_mapping
        self._physical_interface = physical_interface

    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge.

        Target is clamped within the active hinges range.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
        pin = self._hinge_mapping[UUIDKey(active_hinge)]
        inverse = self._inverse_pin.get(pin, False)
        clamped_inversed = (-1.0 if inverse else 1.0) * min(
            max(target, -active_hinge.range), active_hinge.range
        )
        self._physical_interface.set_servo_target(pin=pin, target=clamped_inversed)
