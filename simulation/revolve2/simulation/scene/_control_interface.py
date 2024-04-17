from abc import ABC, abstractmethod

from ._joint_hinge import JointHinge
from ._motor import Motor


class ControlInterface(ABC):
    """Interface for controlling a scene during simulation."""

    @abstractmethod
    def set_joint_hinge_position_target(
        self, joint_hinge: JointHinge, position: float
    ) -> None:
        """
        Set the position target of a hinge joint.

        :param joint_hinge: The hinge to set the position target for.
        :param position: The position target.
        """
        pass

    @abstractmethod
    def set_motor_force(self, motor: Motor, force: float) -> None:
        """
        Set the force of a motor.

        :param motor: The motor to set the force for.
        :param force: The force target.
        """
        pass
