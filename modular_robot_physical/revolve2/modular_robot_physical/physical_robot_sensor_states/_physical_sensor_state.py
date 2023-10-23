from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    ModularRobotSensorState,
)


class _PlaceHolderSensorState(ActiveHingeSensorState):
    @property
    def position(self) -> float:
        """
        Return the position.

        :return: It is a placeholder: 0.0.
        """
        return 0.0


class PhysicalSensorState(ModularRobotSensorState):
    """A Class for using physical sensors."""

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :return: The Sensor State.
        """
        return _PlaceHolderSensorState()
