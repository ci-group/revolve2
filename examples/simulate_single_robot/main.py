"""Main script for the example."""

import pickle

from revolve2.ci_group import terrains
from revolve2.ci_group.simulation import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_physical import Config
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator


def make_body() -> BodyV1:
    """
    Create a body for the robot.

    :returns: The created body.
    """
    # A modular robot body follows a 'tree' structure.
    # The 'Body' class automatically creates a center 'core'.
    # From here, other modular can be attached.
    # Modules can be attach in a rotated fashion.
    # This can be any angle, although the original design takes into account only multiples of 90 degrees.
    body = BodyV1()
    body.core.left = ActiveHingeV1(RightAngles.DEG_0)
    body.core.left.attachment = ActiveHingeV1(RightAngles.DEG_0)
    body.core.left.attachment.attachment = BrickV1(RightAngles.DEG_0)
    body.core.right = ActiveHingeV1(RightAngles.DEG_0)
    body.core.right.attachment = ActiveHingeV1(RightAngles.DEG_0)
    body.core.right.attachment.attachment = BrickV1(RightAngles.DEG_0)
    return body


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up a random number generator, used later.
    rng = make_rng_time_seed()

    # Create a body for the robot.
    body = make_body()
    # Create a brain for the robot.
    # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

    config = Config(
        modular_robot=robot,
        hinge_mapping={},
        run_duration=10,
        control_frequency=10,
        initial_hinge_positions={
            active_hinge: 0.0 for active_hinge in body.find_active_hinges()
        },
    )
    with open("config.pickle", "wb") as f:
        pickle.dump(config, f)

    # Create a modular robot scene.
    # This is a combination of one or more modular robots positioned in a given terrain.
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Create a simulator that will perform the simulation.
    # This tutorial chooses to use Mujoco, but your version of revolve might contain other simulators as well.
    simulator = LocalSimulator()

    # `batch_parameters` are important parameters for simulation.
    # Here, we use the parameters that are standard in CI Group.
    batch_parameters = make_standard_batch_parameters()

    # Simulate the scene.
    # A simulator can run multiple sets of scenes sequentially; it can be reused.
    # However, in this tutorial we only use it once.
    simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )


if __name__ == "__main__":
    main()
