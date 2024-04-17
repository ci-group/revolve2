from abc import ABC, abstractmethod

from .body.base import ActiveHinge, Motor


class ModularRobotControlInterface(ABC):
    """Interface for controlling modular robots."""

    @abstractmethod
    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """

    @abstractmethod
    def set_motor_target(self, motor: Motor, target: float) -> None:
        """
        Set the force target for a motor.

        :param motor: The motor to set the target for.
        :param target: The force target to set.
        """
