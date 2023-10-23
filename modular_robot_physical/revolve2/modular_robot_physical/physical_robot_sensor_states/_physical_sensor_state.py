from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    ModularRobotSensorState,
)
from abc import abstractmethod


class PhysicalSensorState(ModularRobotSensorState):
    """A Class for using physical sensors."""

    @abstractmethod
    def get_active_hinge_sensor_state(
            self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :return: The Sensor State.
        """
        pass
