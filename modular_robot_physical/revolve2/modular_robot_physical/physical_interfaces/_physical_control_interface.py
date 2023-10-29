from abc import abstractmethod
from dataclasses import dataclass

from revolve2.modular_robot import ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge

from .._uuid_key import UUIDKey


@dataclass
class Pin:
    """A wrapper for pins."""

    pin: int
    invert: bool


class PhysicalControlInterface(ModularRobotControlInterface):
    """A base interface for physical robot control."""

    _pins: list[Pin]
    _dry: bool  # if true, gpio output is skipped.
    _debug: bool
    _hinge_mapping: dict[UUIDKey[ActiveHinge], int]
    _inverse_pin: dict[int, bool]

    def __init__(
        self,
        debug: bool,
        dry: bool,
        hinge_mapping: dict[UUIDKey[ActiveHinge], int],
        inverse_pin: dict[int, bool],
    ) -> None:
        """
        Initialize the PhysicalInterface.

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        :param inverse_pin: If certain pins have to be reversed.
        """
        self._inverse_pin = inverse_pin
        self._dry = dry
        self._debug = debug
        self._hinge_mapping = hinge_mapping

    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge.

        Target is clamped within the active hinges range.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
        pin_id = self._hinge_mapping[UUIDKey(active_hinge)]
        pin = Pin(pin=pin_id, invert=self._inverse_pin.get(pin_id, False))
        clamped = min(max(target, -active_hinge.range), active_hinge.range)
        self._set_servo_target(pin=pin, target=clamped)

    @abstractmethod
    def _set_servo_target(self, pin: Pin, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin: The servos pin.
        :param target: The target angle.
        """
