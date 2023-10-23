from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import ActiveHingeSensorState

from ._physical_sensor_state import PhysicalSensorState


class _PlaceholderSensorState(ActiveHingeSensorState):
    """This is a placeholder SensorState until V2 SensorStates are implemented."""

    @property
    def position(self) -> float:
        """
        Return the placeholder position.

        :return: The position.
        """
        return 0.0


class V2PhysicalSensorState(PhysicalSensorState):
    """A Class for using V2 physical sensors."""

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :return: The Sensor State.
        """
        return _PlaceholderSensorState()
