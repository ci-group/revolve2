from abc import ABC, abstractmethod


class PhysicalInterface(ABC):
    """Abstract implementation for interfacing with hardware."""

    @abstractmethod
    def set_servo_target(self, pin: int, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin: The GPIO pin number.
        :param target: The target angle.
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
