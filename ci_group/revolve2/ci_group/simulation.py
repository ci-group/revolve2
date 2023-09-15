"""Standard simulation functions and parameters."""
from revolve2.modular_robot import (
    ModularRobot,
    create_batch_multiple_isolated_robots,
    create_batch_single_robot,
)
from revolve2.simulation import Terrain
from revolve2.simulation.running import Batch
from typing import Type

STANDARD_SIMULATION_TIME = 30
STANDARD_SAMPLING_FREQUENCY = 0.0001
STANDARD_SIMULATION_TIMESTEP = 0.001
STANDARD_CONTROL_FREQUENCY = 60


def create_batch_single_robot_standard(
    robot: Type[ModularRobot],
    terrain: Terrain,
    simulation_time: int | None = STANDARD_SIMULATION_TIME,
    sampling_frequency: float = STANDARD_SAMPLING_FREQUENCY,
    simulation_timestep: float = STANDARD_SIMULATION_TIMESTEP,
    control_frequency: float = STANDARD_CONTROL_FREQUENCY,
) -> Batch:
    """
    Create a simulation batch for a single robot from that robot and a terrain using standard parameters.

    :param robot: The robot to simulate.
    :param terrain: The terrain to put the robot in.
    :param simulation_time: See `Batch` class.
    :param sampling_frequency: See `Batch` class.
    :param simulation_timestep: See `Batch` class.
    :param control_frequency: See `Batch` class.
    :returns: The created batch, ready for simulation.
    """
    return create_batch_single_robot(
        robot=robot,
        terrain=terrain,
        simulation_time=simulation_time,
        sampling_frequency=sampling_frequency,
        simulation_timestep=simulation_timestep,
        control_frequency=control_frequency,
    )


def create_batch_multiple_isolated_robots_standard(
    robots: list[Type[ModularRobot]],
    terrains: list[Terrain],
    simulation_time: int | None = STANDARD_SIMULATION_TIME,
    sampling_frequency: float = STANDARD_SAMPLING_FREQUENCY,
    simulation_timestep: float = STANDARD_SIMULATION_TIMESTEP,
    control_frequency: float = STANDARD_CONTROL_FREQUENCY,
) -> Batch:
    """
    Create a simulation batch for multiple robots that do not interact using standard parameters.

    :param robots: The robots to simulate.
    :param terrains: The terrain to put each robot in.
    :param simulation_time: See `Batch` class.
    :param sampling_frequency: See `Batch` class.
    :param simulation_timestep: See `Batch` class.
    :param control_frequency: See `Batch` class.
    :returns: The created batch, ready for simulation.
    """
    return create_batch_multiple_isolated_robots(
        robots=robots,
        terrains=terrains,
        simulation_time=simulation_time,
        sampling_frequency=sampling_frequency,
        simulation_timestep=simulation_timestep,
        control_frequency=control_frequency,
    )
