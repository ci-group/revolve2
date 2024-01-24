from abc import ABC, abstractmethod

from pyrr import Vector3


class IMUSensorState(ABC):
    """The state of an IMU sensor."""

    @property
    @abstractmethod
    def specific_force(self) -> Vector3:
        """
        Get the measured specific force.

        :returns: The measured specific force.
        """

    @property
    @abstractmethod
    def angular_rate(self) -> Vector3:
        """
        Get the measured angular rate.

        :returns: The measured angular rate.
        """

    @property
    @abstractmethod
    def orientation(self) -> Vector3:
        """
        Get the measured orientation.

        :returns: The measured orientation.
        """
