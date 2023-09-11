from typing import Optional

from revolve2.core.physics import Terrain, create_environment_single_actor
from revolve2.core.physics.running import Batch

from ._modular_robot import ModularRobot


def create_batch_single_robot(
    robot: ModularRobot,
    terrain: Terrain,
    simulation_time: Optional[int],
    sampling_frequency: float,
    simulation_timestep: float,
    control_frequency: float,
) -> Batch:
    """
    Create a batch for simulating a single robot.

    :param robot: The robot to simulate.
    :param terrain: The terrain to simulate the robot in.
    :param simulation_time: See `Batch` class.
    :param sampling_frequency: See `Batch` class.
    :param simulation_timestep: See `Batch` class.
    :param control_frequency: See `Batch` class.
    :returns: The created batch.
    """
    actor, controller = robot.make_actor_and_controller()
    env = create_environment_single_actor(actor, controller, terrain)

    batch = Batch(
        simulation_time=simulation_time,
        sampling_frequency=sampling_frequency,
        simulation_timestep=simulation_timestep,
        control_frequency=control_frequency,
    )
    batch.environments.append(env)

    return batch
