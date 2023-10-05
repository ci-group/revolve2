"""Main script for the example."""

from revolve2.ci_group import terrains
from revolve2.ci_group.simulation import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import ActiveHinge, Body, Brick, RightAngles
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco import LocalSimulator


def make_body() -> Body:
    """
    Create a body for the robot.

    :returns: The created body.
    """
    # A modular robot body follows a 'tree' structure.
    # The 'Body' class automatically creates a center 'core'.
    # From here, other modular can be attached.
    # Modules can be attach in a rotated fashion.
    # This can be any angle, although the original design takes into account only multiples of 90 degrees.
    body = Body()
    body.core.left = ActiveHinge(RightAngles.DEG_0)
    body.core.left.attachment = ActiveHinge(RightAngles.DEG_0)
    body.core.left.attachment.attachment = Brick(RightAngles.DEG_0)
    body.core.right = ActiveHinge(RightAngles.DEG_0)
    body.core.right.attachment = ActiveHinge(RightAngles.DEG_0)
    body.core.right.attachment.attachment = Brick(RightAngles.DEG_0)
    return body


def main() -> None:
    """Run the simulation."""
    # Set up standard logging.
    setup_logging()

    # Set up a random number generator, used later.
    RNG_SEED = 5
    rng = make_rng(RNG_SEED)

    # Create a body for the robot.
    body = make_body()
    # Create a brain for the robot.
    # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

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
