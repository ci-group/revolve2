"""
Simulate modular robot that do not interact, and then calculate their xy displacement.

To understand better, first see the 'evaluate_single_robot' example.

You learn:
- How to simulate multiple isolated robots.
- How to process their simulation results.
"""

import asyncio

from revolve2.ci_group import fitness_functions, modular_robots
from revolve2.ci_group import terrains as standard_terrains
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group.rng import make_rng
from revolve2.ci_group.simulation import create_batch_multiple_isolated_robots_standard
from revolve2.modular_robot.v1 import ModularRobotV1
from revolve2.modular_robot import get_body_states_multiple_isolated_robots
from revolve2.modular_robot.brains import BrainCpgNetworkNeighborRandom
from revolve2.simulators.mujoco import LocalRunner


def main() -> None:
    """Run the simulation."""
    # Set up standard logging.
    setup_logging()

    # Set up a random number generater.
    RNG_SEED = 5
    rng = make_rng(RNG_SEED)

    # Create the robots.
    bodies = [
        modular_robots.gecko(),
        modular_robots.ant(),
        modular_robots.snake(),
        modular_robots.spider(),
    ]
    brains = [BrainCpgNetworkNeighborRandom(rng) for _ in bodies]
    robots = [ModularRobotV1(body, brain) for body, brain in zip(bodies, brains)]

    # Create a flat terrain and copy references to it into an array the size of the number of robots.
    terrain = standard_terrains.flat()
    terrains = [terrain for _ in range(len(robots))]

    # Create the simulation batch.
    batch = create_batch_multiple_isolated_robots_standard(robots, terrains)

    # Create the runner.
    # We set the headless parameters, which will run the simulation as fast as possible.
    # While a runner runs one batch at a time, multiple simulations in that batch might be ran in parallel.
    # In case of the Mujoco runner, see the 'num_simulators' argument.
    # Increasing this number causes it to run more simulations at the same time.
    runner = LocalRunner(headless=True, num_simulators=4)

    # Run the batch and get the results
    results = asyncio.run(runner.run_batch(batch))

    # Get the body states
    body_states = get_body_states_multiple_isolated_robots(bodies, results)

    # Calculate the xy displacement from the body states.
    xy_displacements = [
        fitness_functions.xy_displacement(body_state_begin, body_state_end)
        for body_state_begin, body_state_end in body_states
    ]

    print(xy_displacements)


if __name__ == "__main__":
    main()
