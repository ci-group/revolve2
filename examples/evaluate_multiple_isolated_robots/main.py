"""Main script for the example."""

import logging

from revolve2.ci_group import fitness_functions, modular_robots_v1, terrains
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up the random number generator.
    rng = make_rng_time_seed()

    # Create the robots.
    bodies = [
        modular_robots_v1.gecko_v1(),
        modular_robots_v1.ant_v1(),
        modular_robots_v1.snake_v1(),
        modular_robots_v1.spider_v1(),
    ]
    brains = [BrainCpgNetworkNeighborRandom(body, rng) for body in bodies]
    robots = [ModularRobot(body, brain) for body, brain in zip(bodies, brains)]

    # Create a flat terrain
    terrain = terrains.flat()

    # Create the scenes.
    scenes = []
    for robot in robots:
        scene = ModularRobotScene(terrain=terrain)
        scene.add_robot(robot)
        scenes.append(scene)

    # Create the simulator.
    # A simulator can run multiple scene in parallel.
    # For the MuJoCo simulator, we can control this using the 'num_simulators' argument.
    # Increasing this number causes more simulations to run at the same time.
    simulator = LocalSimulator(headless=True, num_simulators=4)

    # Simulate all scenes.
    scene_states = simulate_scenes(
        simulator=simulator,
        batch_parameters=make_standard_batch_parameters(),
        scenes=scenes,
    )

    # Calculate the xy displacements.
    xy_displacements = [
        fitness_functions.xy_displacement(
            states[0].get_modular_robot_simulation_state(robot),
            states[-1].get_modular_robot_simulation_state(robot),
        )
        for robot, states in zip(robots, scene_states)
    ]

    logging.info(xy_displacements)


if __name__ == "__main__":
    main()
