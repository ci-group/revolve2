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

from .._uuid_key import UUIDKey
from ._active_hinge_sensor_state_impl import ActiveHingeSensorStateImpl
from ._camera_sensor_state_impl import CameraSensorStateImpl
from ._imu_sensor_state_impl import IMUSensorStateImpl


class ModularRobotSensorStateImplV2(ModularRobotSensorState):
    """Implementation of ModularRobotSensorState for v2 robots."""

    _hinge_sensor_mapping: dict[UUIDKey[ActiveHingeSensor], int]
    _hinge_positions: dict[int, float]

    _imu_sensor_states: dict[UUIDKey[IMUSensor], IMUSensorStateImpl]
    _camera_sensor_states: dict[UUIDKey[CameraSensor], CameraSensorStateImpl]

    def __init__(
        self,
        hinge_sensor_mapping: dict[UUIDKey[ActiveHingeSensor], int],
        hinge_positions: dict[int, float],
        imu_sensor_states: dict[UUIDKey[IMUSensor], IMUSensorStateImpl],
        camera_sensor_states: dict[UUIDKey[CameraSensor], CameraSensorStateImpl],
    ) -> None:
        """
        Initialize this object.

        :param hinge_sensor_mapping: Mapping from active hinge sensors to pin ids.
        :param hinge_positions: Position of hinges accessed by pin id.
        :param imu_sensor_states: State of the IMU sensors.
        :param camera_sensor_states: The states of the Camera sensors.
        """
        self._hinge_sensor_mapping = hinge_sensor_mapping
        self._hinge_positions = hinge_positions
        self._imu_sensor_states = imu_sensor_states
        self._camera_sensor_states = camera_sensor_states

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :returns: The Sensor State.
        """
        return ActiveHingeSensorStateImpl(
            self._hinge_positions[self._hinge_sensor_mapping[UUIDKey(sensor)]]
        )

    def get_imu_sensor_state(self, sensor: IMUSensor) -> IMUSensorState:
        """
        Get the state of the provided IMU sensor.

        :param sensor: The sensor.
        :raises ValueError: If IMU sensors is not part of the robot.
        :returns: The state.
        """
        state = self._imu_sensor_states.get(UUIDKey(sensor))
        if state is None:
            raise ValueError(
                "State for IMU sensor not found. Does it exist in the robot definition?"
            )
        return state

    def get_camera_sensor_state(self, sensor: CameraSensor) -> CameraSensorState:
        """
        Get the state of the provided camera sensor.

        :param sensor: The sensor.
        :raises ValueError: If sensor is not found on the robot.
        :returns: The camera sensor state.
        """
        state = self._camera_sensor_states.get(UUIDKey(sensor))
        if state is None:
            raise ValueError(
                "State for camera sensor not found. Does it exist in the robot definition?"
            )
        return state
