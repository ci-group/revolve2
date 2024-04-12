"""Main script for the example."""

import logging

from pyrr import Vector3

from revolve2.ci_group import fitness_functions, modular_robots_v2, terrains
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulation.scene import Pose
from revolve2.simulators.mujoco_simulator import LocalSimulator


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up the random number generator.
    rng = make_rng_time_seed()

    # Create the robots.
    bodies = [
        modular_robots_v2.gecko_v2(),
        modular_robots_v2.ant_v2(),
        modular_robots_v2.snake_v2(),
        modular_robots_v2.spider_v2(),
    ]
    brains = [BrainCpgNetworkNeighborRandom(body, rng) for body in bodies]
    robots = [ModularRobot(body, brain) for body, brain in zip(bodies, brains)]

    """
    Contrary to the previous examples, we now create a single scene and put all robots in it. 
    We place the robots at separate locations in the terrain so they do not overlap at the start of the simulation.
    """
    scene = ModularRobotScene(terrain=terrains.flat())
    poses = [
        Pose(Vector3([1.0, 0.0, 0.0])),
        Pose(Vector3([-1.0, 0.0, 0.0])),
        Pose(Vector3([0.0, 1.0, 0.0])),
        Pose(Vector3([0.0, -1.0, 0.0])),
    ]
    for robot, pose in zip(robots, poses):
        scene.add_robot(robot, pose=pose)

    # Create the simulator.
    simulator = LocalSimulator(headless=False, num_simulators=1)

    # Simulate all scenes.
    scene_states = simulate_scenes(
        simulator=simulator,
        batch_parameters=make_standard_batch_parameters(),
        scenes=scene,
    )

    # Calculate the xy displacements.
    xy_displacements = [
        fitness_functions.xy_displacement(
            scene_states[0].get_modular_robot_simulation_state(robot),
            scene_states[-1].get_modular_robot_simulation_state(robot),
        )
        for robot in robots
    ]

    logging.info(xy_displacements)


if __name__ == "__main__":
    main()
