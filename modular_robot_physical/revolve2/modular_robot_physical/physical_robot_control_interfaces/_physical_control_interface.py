from __future__ import annotations
from revolve2.modular_robot import ModularRobotControlInterface
from abc import ABC, abstractmethod
from dataclasses import dataclass
from revolve2.modular_robot.body.base import ActiveHinge


class PhysicalControlInterface(ABC, ModularRobotControlInterface):
    """A base interface for physical robot control."""
    _pins: list[_Pin]
    _dry: bool  # if true, gpio output is skipped.
    _debug: bool
    _hinge_mapping: dict[ActiveHinge, int]

    def __init__(self, debug: bool, dry: bool, hinge_mapping: dict[ActiveHinge, int]) -> None:
        """
        Initialize the PhysicalInterface.

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        """
        self._dry = dry
        self._debug = debug
        self._hinge_mapping = hinge_mapping

    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
        pin_id = self._hinge_mapping[active_hinge]
        self.set_servo_target(pin_id=pin_id, target=target, careful=False)

    @abstractmethod
    def stop_pwm(self) -> None:
        """Stops the robot."""
        pass

    @abstractmethod
    def init_gpio(self) -> None:
        """Initialize the gpio."""
        pass

    @abstractmethod
    def set_servo_targets(self, targets: list[float], careful: bool = False) -> None:
        """
        Set the targets for servos.

        :param targets: The servos targets.
        :param careful: If careful.
        """
        pass

    @abstractmethod
    def set_servo_target(self, pin_id: int, target: float, careful: bool) -> None:
        """
        Set the target for a single Servo.

        :param pin_id: The servos pin id.
        :param target: The target angle.
        :param careful: If careful.
        """
        pass

    @dataclass
    class _Pin:
        """A wrapper for pins."""
        pin: int
        invert: bool
