import copy

from revolve2.modular_robot.body.sensors import CameraSensor
from revolve2.simulation.scene import MultiBodySystem, Pose, RigidBody, UUIDKey
from revolve2.simulation.scene.sensors import CameraSensor as CameraSim

from .._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from .._unbuilt_child import UnbuiltChild
from ._builder import Builder


class ActiveHingeSensorBuilder(Builder):
    """A Builder for Cores."""

    _sensor: CameraSensor
    _pose: Pose

    def __init__(self, sensor: CameraSensor, rigid_body: RigidBody, pose: Pose) -> None:
        """
        Initialize the Camera Sensor Builder.

        :param sensor: The sensor to be built.
        :param rigid_body: The rigid body for the module to be built on.
        :param pose: The pose of the camera.
        """
        self._sensor = sensor
        self._pose = pose
        self._rigid_body = rigid_body

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
        sensor = CameraSim(pose)

        body_to_multi_body_system_mapping.camera_to_sim_camera[
            UUIDKey(self._sensor)
        ] = sensor
        self._rigid_body.sensors.add_sensor(sensor)
        return []
