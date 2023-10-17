from revolve2.modular_robot.sensor_state import ActiveHingeSensorState
from revolve2.simulation.scene import JointHinge, SimulationState


class ActiveHingeSensorStateImpl(ActiveHingeSensorState):
    """Implements the active hinge sensor state."""

    _simulation_state: SimulationState
    _hinge_joint: JointHinge

    def __init__(
        self, simulation_state: SimulationState, hinge_joint: JointHinge
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param hinge_joint: The hinge joint this state is for.
        """
        self._simulation_state = simulation_state
        self._hinge_joint = hinge_joint

    @property
    def position(self) -> float:
        """
        Get the measured position of the active hinge.

        :returns: The measured position.
        """
        return self._simulation_state.get_hinge_joint_position(self._hinge_joint)
