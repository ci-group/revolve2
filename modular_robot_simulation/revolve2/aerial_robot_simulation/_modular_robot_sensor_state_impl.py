from revolve2.aerial_robot.body.base import MotorSensor
from revolve2.aerial_robot.sensor_state import (
    MotorSensorState,
    AerialRobotSensorState,
)
from revolve2.simulation.scene import SimulationState, UUIDKey

from ._motor_sensor_state_impl import MotorSensorStateImpl
from ._build_multi_body_systems import BodyToMultiBodySystemMapping


class ModularRobotSensorStateImpl(AerialRobotSensorState):
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

    def get_motor_sensor_state(
        self, sensor: MotorSensor
    ) -> MotorSensorState:
        """
        Get the state of the provided motor sensor.

        :param sensor: The sensor.
        :returns: The state.
        """
        maybe_motor = self._body_to_multi_body_system_mapping.motor_sensor_to_sim_motor.get(
            UUIDKey(sensor)
        )
        assert maybe_motor is not None, "Sensor not in scene."

        return MotorSensorStateImpl(
            simulation_state=self._simulation_state, motor=maybe_motor
        )
