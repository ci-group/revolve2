from revolve2.aerial_robot import AerialRobot
from revolve2.simulation.scene import MultiBodySystem, SimulationState, UUIDKey

from ._modular_robot_simulation_state import ModularRobotSimulationState


class SceneSimulationState:
    """An interface for reading a certain state of a modular robot simulation."""

    _simulation_state: SimulationState
    _modular_robot_to_multi_body_system_mapping: dict[
        UUIDKey[AerialRobot], MultiBodySystem
    ]

    def __init__(
        self,
        simulation_state: SimulationState,
        modular_robot_to_multi_body_system_mapping: dict[
            UUIDKey[AerialRobot], MultiBodySystem
        ],
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The simulation state corresponding to this modular robot scene state.
        :param modular_robot_to_multi_body_system_mapping: A mapping from modular robots to multi-body systems.
        """
        self._simulation_state = simulation_state
        self._modular_robot_to_multi_body_system_mapping = (
            modular_robot_to_multi_body_system_mapping
        )

    def get_modular_robot_simulation_state(
        self, modular_robot: AerialRobot
    ) -> ModularRobotSimulationState:
        """
        Get the simulation state for one of the modular robots in the scene.

        :param modular_robot: The modular robot to get the state for.
        :returns: The retrieved state.
        :raises ValueError: If the robot is not in the scene.
        """
        maybe_multi_body_system = self._modular_robot_to_multi_body_system_mapping.get(
            UUIDKey(modular_robot)
        )
        if maybe_multi_body_system is None:
            raise ValueError("Modular robot not in scene.")

        return ModularRobotSimulationState(
            self._simulation_state, maybe_multi_body_system
        )
