"""
Simulate a single modular robot, and then calculate its xy displacement.

To understand simulation, first see the 'simulate_single_robot' example.

You learn:
- How to process simulation results.
"""

import asyncio

from revolve2.core.modular_robot import ModularRobot, get_body_states_single_robot
from revolve2.core.modular_robot.brains import BrainCpgNetworkNeighborRandom
from revolve2.runners.mujoco import LocalRunner
from revolve2.standard_resources import fitness_functions, modular_robots, terrains
from revolve2.standard_resources.rng import make_rng
from revolve2.standard_resources.simulation import create_batch_single_robot_standard


def main() -> None:
    """Run the simulation."""
    # Set up a random number generater.
    RNG_SEED = 5
    rng = make_rng(RNG_SEED)

    # Create the robot.
    body = modular_robots.gecko()
    brain = BrainCpgNetworkNeighborRandom(rng)
    robot = ModularRobot(body, brain)

    # Create the simulation batch.
    batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.flat())

    # Create the runner.
    # We set the headless parameters, which will run the simulation as fast as possible.
    runner = LocalRunner(headless=True)
    # Running the batch returns simulation results.
    results = asyncio.run(runner.run_batch(batch))

    # Get the results of the first environment, which is the only one since we set up a single simulation environment.
    environment_results = results.environment_results[0]

    # We have to map the simulation results back to robot body space.
    # This function calculates the state of the robot body at the start and end of the simulation.
    body_state_begin, body_state_end = get_body_states_single_robot(
        body, environment_results
    )

    # Calculate the xy displacement from the body states.
    xy_displacement = fitness_functions.xy_displacement(
        body_state_begin, body_state_end
    )

    print(xy_displacement)


if __name__ == "__main__":
    main()
