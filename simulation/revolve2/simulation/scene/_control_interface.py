from abc import ABC, abstractmethod

from ._joint_hinge import JointHinge


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
