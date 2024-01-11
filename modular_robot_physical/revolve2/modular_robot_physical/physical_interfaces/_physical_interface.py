from abc import ABC, abstractmethod
from typing import Sequence


class PhysicalInterface(ABC):
    """Abstract implementation for interfacing with hardware."""

    @abstractmethod
    def set_servo_targets(self, pins: list[int], targets: list[float]) -> None:
        """
        Set the target for multiple servos.

        This can be a fairly slow operation.

        :param pins: The GPIO pin numbers.
        :param targets: The target angles.
        """

    @abstractmethod
    def enable(self) -> None:
        """Start the robot."""

    @abstractmethod
    def disable(self) -> None:
        """
        Set the robot to low power mode.

        This disables all active modules and sensors.
        """

    @abstractmethod
    def get_battery_level(self) -> float:
        """
        Get the battery level.

        :returns: The battery level as a number between 0.0 and 1.0.
        :raises NotImplementedError: If getting the battery level is not supported on this hardware.
        """

    @abstractmethod
    def get_multiple_servo_positions(self, pins: Sequence[int]) -> list[float]:
        """
        Get the current position of multiple servos.

        :param pins: The GPIO pin numbers.
        :returns: The current positions.
        :raises NotImplementedError: If getting the servo position is not supported on this hardware.
        """
