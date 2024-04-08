from abc import ABC, abstractmethod

from .body.base import Motor


class DroneControlInterface(ABC):
    """Interface for controlling aerial robots."""

    @abstractmethod
    def set_motor_target(self, motor: Motor, target: float) -> None:
        """
        Set the position target for an active hinge.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
