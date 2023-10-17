import mujoco
import numpy as np
import numpy.typing as npt
from pyrr import Quaternion, Vector3

from revolve2.simulation.scene import (
    JointHinge,
    MultiBodySystem,
    Pose,
    RigidBody,
    SimulationState,
    UUIDKey,
)

from ._abstraction_to_mujoco_mapping import AbstractionToMujocoMapping


class SimulationStateImpl(SimulationState):
    """Implementation of the simulation state interface for MuJoCo."""

    _xpos: npt.NDArray[np.float_]
    _xquat: npt.NDArray[np.float_]
    _qpos: npt.NDArray[np.float_]
    _abstraction_to_mujoco_mapping: AbstractionToMujocoMapping

    def __init__(
        self,
        data: mujoco.MjData,
        abstraction_to_mujoco_mapping: AbstractionToMujocoMapping,
    ) -> None:
        """
        Initialize this object.

        The copies required information from the provided data.
        As such the data can be modified after this constructor without causing problems.

        :param data: The data to copy from.
        :param abstraction_to_mujoco_mapping: A mapping between simulation abstraction and mujoco.
        """
        self._xpos = data.xpos.copy()
        self._xquat = data.xquat.copy()
        self._qpos = data.qpos.copy()
        self._abstraction_to_mujoco_mapping = abstraction_to_mujoco_mapping

    def get_rigid_body_relative_pose(self, rigid_body: RigidBody) -> Pose:
        """
        Get the pose of a rigid body, relative to its parent multi-body system's reference frame.

        :param rigid_body: The rigid body to get the pose for.
        :returns: The relative pose.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
        return Pose()

    def get_rigid_body_absolute_pose(self, rigid_body: RigidBody) -> Pose:
        """
        Get the pose of a rigid body, relative the global reference frame.

        :param rigid_body: The rigid body to get the pose for.
        :returns: The absolute pose.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
        return Pose()

    def get_multi_body_system_pose(self, multi_body_system: MultiBodySystem) -> Pose:
        """
        Get the pose of a multi-body system, relative to the global reference frame.

        :param multi_body_system: The multi-body system to get the pose for.
        :returns: The relative pose.
        """
        body_mujoco = self._abstraction_to_mujoco_mapping.multi_body_system[
            UUIDKey(multi_body_system)
        ]
        pose = Pose(
            Vector3(self._xpos[body_mujoco.id]), Quaternion(self._xquat[body_mujoco.id])
        )
        return pose

    def get_hinge_joint_position(self, joint: JointHinge) -> float:
        """
        Get the rotational position of a hinge joint.

        :param joint: The joint to get the rotational position for.
        :returns: The rotational position.
        """
        joint_mujoco = self._abstraction_to_mujoco_mapping.hinge_joint[UUIDKey(joint)]
        return float(self._qpos[joint_mujoco.id])
