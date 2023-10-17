from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    ModularRobotSensorState,
)
from revolve2.simulation.scene import SimulationState, UUIDKey

from ._active_hinge_sensor_state_impl import ActiveHingeSensorStateImpl
from ._build_multi_body_systems import BodyToMultiBodySystemMapping


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
