from typing import overload

from revolve2.simulation.simulator import BatchParameters, RecordSettings, Simulator

from ._modular_robot_scene import ModularRobotScene
from ._to_batch import to_batch
from .simulation_states import SceneSimulationState


@overload
def simulate_scenes(
    simulator: Simulator,
    batch_parameters: BatchParameters,
    scenes: ModularRobotScene,
    record_settings: RecordSettings | None = None,
) -> list[SceneSimulationState]:
    """
    Simulate a scene.

    :param simulator: The simulator to use for simulation.
    :param batch_parameters: The batch parameters to use for simulation.
    :param scenes: Te scene to simulate.
    :param record_settings: The optional record settings to use during simulation.
    :returns: A list of simulation states.

    # noqa: DAR202
    # Darglint complains about no return statement, but this is an overload stub so we can safely ignore that.
    """


@overload
def simulate_scenes(
    simulator: Simulator,
    batch_parameters: BatchParameters,
    scenes: list[ModularRobotScene],
    record_settings: RecordSettings | None = None,
) -> list[list[SceneSimulationState]]:
    """
    Simulate multiple scenes.

    :param simulator: The simulator to use for simulation.
    :param batch_parameters: The batch parameters to use for simulation.
    :param scenes: The scenes to simulate.
    :param record_settings: The optional record settings to use during simulation.
    :returns: A list of simulation states for each scene in the provided batch.

    # noqa: DAR202
    # Darglint complains about no return statement, but this is an overload stub so we can safely ignore that.
    """


def simulate_scenes(
    simulator: Simulator,
    batch_parameters: BatchParameters,
    scenes: ModularRobotScene | list[ModularRobotScene],
    record_settings: RecordSettings | None = None,
) -> list[SceneSimulationState] | list[list[SceneSimulationState]]:
    """
    Simulate one or more scenes.

    :param simulator: The simulator to use for simulation.
    :param batch_parameters: The batch parameters to use for simulation.
    :param scenes: One or more scenes to simulate.
    :param record_settings: The optional record settings to use during simulation.
    :returns: A list of simulation states for each scene in the provided batch.
    """
    if isinstance(scenes, ModularRobotScene):
        return_scalar_result = True
        scenes = [scenes]
    else:
        return_scalar_result = False

    batch, modular_robot_to_multi_body_system_mappings, interactive_objects_batch = (
        to_batch(scenes, batch_parameters, record_settings)
    )
    simulation_results = simulator.simulate_batch(batch)

    results = [
        [
            SceneSimulationState(
                state, modular_robot_to_multi_body_system_mapping, interactive_objects
            )
            for state in simulation_result
        ]
        for simulation_result, modular_robot_to_multi_body_system_mapping, interactive_objects in zip(
            simulation_results,
            modular_robot_to_multi_body_system_mappings,
            interactive_objects_batch,
            strict=True,
        )
    ]

    return results[0] if return_scalar_result else results
