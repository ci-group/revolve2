from revolve2.modular_robot.body.sensors import (
    ActiveHingeSensor,
    CameraSensor,
    IMUSensor,
)
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    CameraSensorState,
    IMUSensorState,
    ModularRobotSensorState,
)
from revolve2.simulation.scene import SimulationState, UUIDKey

from .._build_multi_body_systems import BodyToMultiBodySystemMapping
from . import CameraSensorStateImpl
from ._active_hinge_sensor_state_impl import ActiveHingeSensorStateImpl
from ._imu_sensor_state_impl import IMUSensorStateImpl


class ModularRobotSensorStateImpl(ModularRobotSensorState):
    """Implementation for ModularRobotSensorState."""

    _simulation_state: SimulationState
    _body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping

    def __init__(
        self,
        simulation_state: SimulationState,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        """
        self._simulation_state = simulation_state
        self._body_to_multi_body_system_mapping = body_to_multi_body_system_mapping

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get the state of the provided active hinge sensor.

        :param sensor: The sensor.
        :returns: The state.
        """
        maybe_joint = self._body_to_multi_body_system_mapping.active_hinge_sensor_to_joint_hinge.get(
            UUIDKey(sensor)
        )
        assert maybe_joint is not None, "Sensor not in scene."

        return ActiveHingeSensorStateImpl(
            simulation_state=self._simulation_state, hinge_joint=maybe_joint
        )

    def get_imu_sensor_state(self, sensor: IMUSensor) -> IMUSensorState:
        """
        Get the state of the provided IMU sensor.

        :param sensor: The IMU sensor.
        :returns: The IMU sensor state.
        """
        maybe_imu = self._body_to_multi_body_system_mapping.imu_to_sim_imu.get(
            UUIDKey(sensor)
        )
        assert maybe_imu is not None, "IMU not in scene."

        return IMUSensorStateImpl(
            simulation_state=self._simulation_state,
            multi_body_system=self._body_to_multi_body_system_mapping.multi_body_system,
            imu=maybe_imu,
        )

    def get_camera_sensor_state(self, sensor: CameraSensor) -> CameraSensorState:
        """
        Get the state of the camera sensor.

        :param sensor: The camera sensor.
        :returns: The camera sensor state.
        """
        maybe_camera = self._body_to_multi_body_system_mapping.camera_to_sim_camera.get(
            UUIDKey(sensor)
        )
        assert maybe_camera is not None, "Camera not in scene."

        return CameraSensorStateImpl(
            simulation_state=self._simulation_state,
            multi_body_system=self._body_to_multi_body_system_mapping.multi_body_system,
            camera=maybe_camera,
        )
