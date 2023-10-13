"""Main script for the example."""
import argparse

from revolve2.ci_group import fitness_functions, modular_robots, terrains
from revolve2.ci_group.simulation import (
    STANDARD_SIMULATION_TIME,
    make_standard_batch_parameters,
)
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco import LocalSimulator


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--simulation-time",
        "-t",
        type=int,
        default=STANDARD_SIMULATION_TIME,
        help="Number of seconds to run simulation.",
    )
    args = parser.parse_args()

    # Set up a random number generator.
    rng = make_rng_time_seed()

    # Create the robot.
    body = modular_robots.gecko()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Create the simulator.
    # We set enable the headless flag, which will prevent visualization of the simulation, speeding it up.
    simulator = LocalSimulator(headless=False)
    batch_parameters = make_standard_batch_parameters(
        simulation_time=args.simulation_time
    )

    # Obtain the state of the simulation, measured at a predefined interval as defined in the batch parameters.
    scene_states = simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )

    # Get the state at the begin and end of the simulation.
    scene_state_begin = scene_states[0]
    scene_state_end = scene_states[-1]

    # Retrieve the state of the modular robot.
    robot_state_begin = scene_state_begin.get_modular_robot_simulation_state(robot)
    robot_state_end = scene_state_end.get_modular_robot_simulation_state(robot)

    # Calculate the xy displacement.
    xy_displacement = fitness_functions.xy_displacement(
        robot_state_begin, robot_state_end
    )

    print(xy_displacement)


if __name__ == "__main__":
    main()
