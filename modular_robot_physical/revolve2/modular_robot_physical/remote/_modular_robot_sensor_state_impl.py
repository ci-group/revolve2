from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    ModularRobotSensorState,
)


class ModularRobotSensorStateImpl(ModularRobotSensorState):
    """Implementation of ModularRobotSensorState."""

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :returns: The Sensor State.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
        return ActiveHingeSensorState()
