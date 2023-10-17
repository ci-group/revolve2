import mujoco
import numpy as np
import numpy.typing as npt
from pyrr import Quaternion, Vector3

from revolve2.simulation.scene import (
    MultiBodySystem,
    Pose,
    RigidBody,
    SimulationState,
    UUIDKey,
)

from ._body_id import BodyId


class SimulationStateImpl(SimulationState):
    """Implementation of the simulation state interface for MuJoCo."""

    _xpos: npt.NDArray[np.float_]
    _xquat: npt.NDArray[np.float_]
    _multi_body_system_to_mujoco_body_id_mapping: dict[UUIDKey[MultiBodySystem], BodyId]

    def __init__(
        self,
        data: mujoco.MjData,
        multi_body_system_to_mujoco_body_id_mapping: dict[
            UUIDKey[MultiBodySystem], BodyId
        ],
    ) -> None:
        """
        Initialize this object.

        The copies required information from the provided data.
        As such the data can be modified after this constructor without causing problems.

        :param data: The data to copy from.
        :param multi_body_system_to_mujoco_body_id_mapping: A mapping from multi-body systems to mujoco body id.
        """
        self._xpos = data.xpos.copy()
        self._xquat = data.xquat.copy()
        self._multi_body_system_to_mujoco_body_id_mapping = (
            multi_body_system_to_mujoco_body_id_mapping
        )

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
        body_id = self._multi_body_system_to_mujoco_body_id_mapping[
            UUIDKey(multi_body_system)
        ]
        pose = Pose(
            Vector3(self._xpos[body_id.id]), Quaternion(self._xquat[body_id.id])
        )
        return pose
