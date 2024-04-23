from abc import ABC, abstractmethod

from .body.base import ActiveHinge


class ModularRobotControlInterface(ABC):
    """Interface for controlling modular robots."""

    @abstractmethod
    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge on the modular robot.

        :param active_hinge: The active hinge object to set the target for.
        :param target: The target value to set.
        """
