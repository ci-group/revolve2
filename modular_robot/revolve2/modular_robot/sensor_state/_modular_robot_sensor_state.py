from abc import ABC, abstractmethod

from ..body.sensors import ActiveHingeSensor, CameraSensor, IMUSensor
from ._active_hinge_sensor_state import ActiveHingeSensorState
from ._camera_sensor_state import CameraSensorState
from ._imu_sensor_state import IMUSensorState


class ModularRobotSensorState(ABC):
    """The state of modular robot's sensors."""

    @abstractmethod
    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get the state of the provided active hinge sensor.

        :param sensor: The sensor.
        :returns: The state.
        """

    @abstractmethod
    def get_imu_sensor_state(self, sensor: IMUSensor) -> IMUSensorState:
        """
        Get the state of the provided IMU sensor.

        :param sensor: The sensor.
        :returns: The state.
        """

    @abstractmethod
    def get_camera_sensor_state(self, sensor: CameraSensor) -> CameraSensorState:
        """
        Get the state of the provided camera sensor.

        :param sensor: The sensor.
        :returns: The state.
        """
