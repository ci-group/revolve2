from abc import ABC, abstractmethod


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
