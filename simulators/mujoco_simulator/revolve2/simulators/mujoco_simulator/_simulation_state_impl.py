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
from revolve2.simulation.scene.sensors import CameraSensor, IMUSensor

from ._abstraction_to_mujoco_mapping import AbstractionToMujocoMapping


class SimulationStateImpl(SimulationState):
    """Implementation of the simulation state interface for MuJoCo."""

    _xpos: npt.NDArray[np.float_]
    _xquat: npt.NDArray[np.float_]
    _qpos: npt.NDArray[np.float_]
    _sensordata: npt.NDArray[np.float_]
    _abstraction_to_mujoco_mapping: AbstractionToMujocoMapping
    _camera_views: dict[int, npt.NDArray[np.uint8]]

    def __init__(
        self,
        data: mujoco.MjData,
        abstraction_to_mujoco_mapping: AbstractionToMujocoMapping,
        camera_views: dict[int, npt.NDArray[np.uint8]],
    ) -> None:
        """
        Initialize this object.

        The copies required information from the provided data.
        As such the data can be modified after this constructor without causing problems.

        :param data: The data to copy from.
        :param abstraction_to_mujoco_mapping: A mapping between simulation abstraction and mujoco.
        :param camera_views: The camera views.
        """
        self._xpos = data.xpos.copy()
        self._xquat = data.xquat.copy()
        self._qpos = data.qpos.copy()
        self._sensordata = data.sensordata.copy()
        self._abstraction_to_mujoco_mapping = abstraction_to_mujoco_mapping
        self._camera_views = camera_views

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

    def get_imu_specific_force(self, imu_sensor: IMUSensor) -> Vector3:
        """
        Get the specific force measured an IMU.

        :param imu_sensor: The IMU.
        :returns: The specific force.
        """
        accelerometer_id = self._abstraction_to_mujoco_mapping.imu_sensor[
            UUIDKey(imu_sensor)
        ].accelerometer_id
        specific_force = self._sensordata[accelerometer_id : accelerometer_id + 3]
        return Vector3(specific_force)

    def get_imu_angular_rate(self, imu_sensor: IMUSensor) -> Vector3:
        """
        Get the angular rate measured by am IMU.

        :param imu_sensor: The IMU.
        :returns: The angular rate.
        """
        gyro_id = self._abstraction_to_mujoco_mapping.imu_sensor[
            UUIDKey(imu_sensor)
        ].gyro_id
        angular_rate = self._sensordata[gyro_id : gyro_id + 3]
        return Vector3(angular_rate)

    def get_camera_view(self, camera_sensor: CameraSensor) -> npt.NDArray[np.uint8]:
        """
        Get the current view of the camera.

        :param camera_sensor: The camera.
        :return: The image (RGB).
        """
        camera_id = self._abstraction_to_mujoco_mapping.camera_sensor[
            UUIDKey(camera_sensor)
        ].camera_id
        image = self._camera_views[camera_id]
        return image
