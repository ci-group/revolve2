import copy

from pyrr import Vector3
from revolve2.modular_robot.body.sensors import IMUSensor
from revolve2.simulation.scene import MultiBodySystem, Pose, RigidBody, UUIDKey
from revolve2.simulation.scene.sensors import IMUSensor as IMUSim

from .._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from .._unbuilt_child import UnbuiltChild
from ._builder import Builder


class IMUSensorBuilder(Builder):
    """A Builder for Cores."""

    _sensor: IMUSensor
    _pose: Pose
    _rigid_body: RigidBody

    def __init__(
        self,
        sensor: IMUSensor,
        rigid_body: RigidBody,
        pose: Pose,
        imu_location: Vector3,
    ) -> None:
        """
        Initialize the IMU-Sensor Builder.

        :param sensor: The sensor to be built.
        :param rigid_body: The rigid body for the module to be built on.
        :param pose: The pose of the sensor.
        :param imu_location: The location of the IMU locally.
        """
        self._sensor = sensor
        self._rigid_body = rigid_body
        self._pose = pose
        self._imu_location = imu_location

    def build(
        self,
        multi_body_system: MultiBodySystem,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> list[UnbuiltChild]:
        """
        Build a module onto the Robot.

        :param multi_body_system: The multi body system of the robot.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        :return: The next children to be built.
        """
        pose = copy.deepcopy(self._pose)
        pose.position += self._imu_location
        sensor = IMUSim(pose=pose)
        body_to_multi_body_system_mapping.imu_to_sim_imu[UUIDKey(self._sensor)] = sensor
        self._rigid_body.sensors.add_sensor(sensor)
        return []
