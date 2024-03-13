from revolve2.aerial_robot.sensor_state import MotorSensorState
from revolve2.simulation.scene import SimMotor, SimulationState


class MotorSensorStateImpl(MotorSensorState):
    """Implements the active hinge sensor state."""

    _simulation_state: SimulationState
    _motor: SimMotor

    def __init__(
        self, simulation_state: SimulationState, motor: SimMotor
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param motor: The motor this state is for.
        """
        self._simulation_state = simulation_state
        self._motor = motor

    @property
    def position(self) -> float:
        """
        Get the measured position of the active hinge.

        :returns: The measured position.
        """
        return self._simulation_state.get_motor_position(self._motor)
