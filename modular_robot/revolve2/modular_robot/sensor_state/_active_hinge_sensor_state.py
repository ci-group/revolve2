from abc import ABC, abstractmethod


class ActiveHingeSensorState(ABC):
    """The state of an active hinge sensor."""

    @property
    @abstractmethod
    def position(self) -> float:
        """
        Get the measured position of the active hinge.

        :returns: The measured position.
        """
