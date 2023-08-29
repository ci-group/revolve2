"""
Simulate and visualize a single modular robot.

You learn:
- How to create a robot body with a basic controller.
- How to simulate and see a robot.
"""

import asyncio

from revolve2.core.modular_robot import (
    ActiveHinge,
    Body,
    Brick,
    ModularRobot,
    RightAngles,
)
from revolve2.core.modular_robot.brains import BrainCpgNetworkNeighborRandom
from revolve2.runners.mujoco import LocalRunner
from revolve2.standard_resources import terrains
from revolve2.standard_resources.rng import make_rng
from revolve2.standard_resources.simulation import create_batch_single_robot_standard


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
    body.core.left = ActiveHinge(RightAngles.DEG_180)
    body.core.left.attachment = ActiveHinge(RightAngles.DEG_180)
    body.core.left.attachment.attachment = Brick(RightAngles.DEG_0)
    body.core.right = ActiveHinge(RightAngles.DEG_180)
    body.core.right.attachment = ActiveHinge(RightAngles.DEG_180)
    body.core.right.attachment.attachment = Brick(RightAngles.DEG_0)
    # A body needs to be finalized after building.
    # This sets some variables on each module that makes it easier to work with the body later.
    # Don't worry if you forget to the finalize; the framework will raise an error when you attempt to perform an action that required finalization beforehand.
    body.finalize()
    return body


def main() -> None:
    """Run the simulation."""
    # Set up a random number generater, used later.
    RNG_SEED = 5
    rng = make_rng(RNG_SEED)

    # Create a body for the robot.
    body = make_body()
    # Create a brain for the robot.
    # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
    brain = BrainCpgNetworkNeighborRandom(rng)
    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

    # Create a batch containing the robot in a flat terrain.
    # A batch describes a set of simulations to perform.
    # In this case there is a single simulation containing a single robot.
    # We use the 'standard' parameters for robot simulation.
    # You probably don't have to worry about this, but if you are interested, take a look inside the function.
    batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.flat())

    # Create a runner that will perform the simulation.
    # This tutorial chooses to use Mujoco, but your version of revolve might contain other simulators as well.
    runner = LocalRunner()
    # Here we use the runner to run a batch.
    # Once a runner finishes a batch, it can be reused to run a new batch.
    # (We only run one batch in this tutorial, so we only use the runner once.)
    asyncio.run(runner.run_batch(batch))


if __name__ == "__main__":
    main()
