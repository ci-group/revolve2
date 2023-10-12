from abc import ABC, abstractmethod

from revolve2.simulation.scene import JointHinge, MultiBodySystem

from .._modular_robot_active_hinge_key import ModularRobotActiveHingeKey
from ._unbuilt_child import UnbuiltChild


class Builder(ABC):
    """An abstract builder class."""

    @abstractmethod
    def build(
        self,
        multi_body_system: MultiBodySystem,
        joint_mapping: dict[ModularRobotActiveHingeKey, JointHinge],
    ) -> list[UnbuiltChild]:
        """
        Build a module onto the Robot.

        :param multi_body_system: The multi body system of the robot.
        :param joint_mapping: The joint mapping of the robot.
        :return: The next children to be built.
        """
