from abc import ABC, abstractmethod

from ._joint_hinge import JointHinge
from ._multi_body_system import MultiBodySystem
from ._pose import Pose
from ._rigid_body import RigidBody


class SimulationState(ABC):
    """Interface for the state of a simulation at certain point."""

    @abstractmethod
    def get_rigid_body_relative_pose(self, rigid_body: RigidBody) -> Pose:
        """
        Get the pose of a rigid body, relative to its parent multi-body system's reference frame.

        :param rigid_body: The rigid body to get the pose for.
        :returns: The relative pose.
        """

    @abstractmethod
    def get_rigid_body_absolute_pose(self, rigid_body: RigidBody) -> Pose:
        """
        Get the pose of a rigid body, relative the global reference frame.

        :param rigid_body: The rigid body to get the pose for.
        :returns: The absolute pose.
        """

    @abstractmethod
    def get_multi_body_system_pose(self, multi_body_system: MultiBodySystem) -> Pose:
        """
        Get the pose of a multi-body system, relative to the global reference frame.

        :param multi_body_system: The multi-body system to get the pose for.
        :returns: The relative pose.
        """

    @abstractmethod
    def get_hinge_joint_position(self, joint: JointHinge) -> float:
        """
        Get the rotational position of a hinge joint.

        :param joint: The joint to get the rotational position for.
        :returns: The rotational position.
        """
