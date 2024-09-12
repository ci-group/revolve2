"""Main script for the example."""

import logging

from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import fitness_functions, modular_robots_v2, terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up a random number generator.
    rng = make_rng_time_seed()

    # Create the robot.
    body = modular_robots_v2.gecko_v2()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng, passive_connections=False)
    robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Create the simulator.
    # We set enable the headless flag, which will prevent visualization of the simulation, speeding it up.
    simulator = LocalSimulator(headless=False)
    batch_parameters = make_standard_batch_parameters()

    # Obtain the state of the simulation, measured at a predefined interval as defined in the batch parameters.
    scene_states = simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )

    """
    Using the previously obtained scene_states we can now start to evaluate our robot.
    Note in this example we simply use x-y displacement, but you can do any other way of evaluation as long as the required data is in the scene states.
    """
    # Get the state at the beginning and end of the simulation.
    scene_state_begin = scene_states[0]
    scene_state_end = scene_states[-1]

    # Retrieve the state of the modular robot, which also contains the location of the robot.
    robot_state_begin = scene_state_begin.get_modular_robot_simulation_state(robot)
    robot_state_end = scene_state_end.get_modular_robot_simulation_state(robot)

    # Calculate the xy displacement, using the locations of the robot.
    xy_displacement = fitness_functions.xy_displacement(
        robot_state_begin, robot_state_end
    )

    logging.info(xy_displacement)


if __name__ == "__main__":
    main()
