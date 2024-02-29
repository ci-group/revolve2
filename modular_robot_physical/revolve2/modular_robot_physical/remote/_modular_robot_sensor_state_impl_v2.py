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


class ModularRobotSensorStateImplV2(ModularRobotSensorState):
    """Implementation of ModularRobotSensorState for v2 robots."""

    _hinge_sensor_mapping: dict[UUIDKey[ActiveHingeSensor], int]
    _positions: dict[int, float]

    def __init__(
        self,
        hinge_sensor_mapping: dict[UUIDKey[ActiveHingeSensor], int],
        positions: dict[int, float],
    ) -> None:
        """
        Initialize this object.

        :param hinge_sensor_mapping: Mapping from active hinge sensors to pin ids.
        :param positions: Position of hinges accessed by pin id.
        """
        self._hinge_sensor_mapping = hinge_sensor_mapping
        self._positions = positions

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :returns: The Sensor State.
        """
        return ActiveHingeSensorStateImpl(
            self._positions[self._hinge_sensor_mapping[UUIDKey(sensor)]]
        )

    def get_imu_sensor_state(self, sensor: IMUSensor) -> IMUSensorState:
        """
        Get the state of the provided IMU sensor.

        :param sensor: The sensor.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()

    def get_camera_sensor_state(self, sensor: CameraSensor) -> CameraSensorState:
        """
        Get the state of the camera sensor.

        :param sensor: The camera.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
