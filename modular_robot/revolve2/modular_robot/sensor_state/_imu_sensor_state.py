from abc import ABC, abstractmethod


class IMUSensorState(ABC):
    """The state of an IMU sensor."""

    @property
    @abstractmethod
    def specific_forace(self) -> float:
        """
        Get the measured specific force.

        :returns: The measured specific force.
        """

    @property
    @abstractmethod
    def angular_rate(self) -> float:
        """
        Get the measured angular rate.

        :returns: The measured angular rate.
        """

    @property
    @abstractmethod
    def orientation(self) -> float:
        """
        Get the measured orientation.

        :returns: The measured orientation.
        """
