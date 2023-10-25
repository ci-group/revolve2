from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import ActiveHingeSensorState

from .._physical_sensor_state import PhysicalSensorState


class _V2ActiveHingeSensorState(ActiveHingeSensorState):
    """Implements ActiveHingeSensorState for v2 hardware."""

    @property
    def position(self) -> float:
        """
        Return the placeholder position.

        :return: The position.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
        return 0.0


class V2PhysicalSensorState(PhysicalSensorState):
    """Implementes PhysicalSensorState for v2 hardware."""

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :return: The Sensor State.
        """
        return _V2ActiveHingeSensorState()
