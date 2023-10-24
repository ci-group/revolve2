from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass

from revolve2.modular_robot import ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge


@dataclass
class _Pin:
    """A wrapper for pins."""

    pin: int
    invert: bool


class PhysicalControlInterface(ModularRobotControlInterface):
    """A base interface for physical robot control."""

    _pins: list[_Pin]
    _dry: bool  # if true, gpio output is skipped.
    _debug: bool
    _hinge_mapping: dict[ActiveHinge, int]
    _inverse_pin: dict[int, bool]

    careful: bool = False

    def __init__(
        self,
        debug: bool,
        dry: bool,
        hinge_mapping: dict[ActiveHinge, int],
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

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
        try:
            pin_id = self._hinge_mapping[active_hinge]
            pin = _Pin(pin=pin_id, invert=self._inverse_pin[pin_id])
            self.set_servo_target(pin=pin, target=target)
        except KeyError:
            pass

    @abstractmethod
    def stop_pwm(self) -> None:
        """Stop the robot."""
        pass

    @abstractmethod
    def set_servo_targets(self, targets: list[float]) -> None:
        """
        Set the targets for servos.

        :param targets: The servos targets.
        """
        pass

    @abstractmethod
    def set_servo_target(self, pin: _Pin, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin: The servos pin.
        :param target: The target angle.
        """
        pass
