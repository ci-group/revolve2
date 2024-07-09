from revolve2.simulation.scene import MultiBodySystem, Pose, SimulationState


class MultiBodySystemSimulationState:
    """The state of a multi body system at some moment in a simulation."""

    _simulation_state: SimulationState
    _multi_body_system: MultiBodySystem
    """The multi-body system corresponding to the multi body system."""

    def __init__(
        self, simulation_state: SimulationState, multi_body_system: MultiBodySystem
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The simulation state corresponding to this multi body systems state.
        :param multi_body_system: The multi-body system this multi body system corresponds to.
        """
        self._simulation_state = simulation_state
        self._multi_body_system = multi_body_system

    def get_pose(self) -> Pose:
        """
        Get the pose of the multi body system.

        :returns: The retrieved pose.
        """
        return self._simulation_state.get_multi_body_system_pose(
            self._multi_body_system
        )
