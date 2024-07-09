"""Main script for the example."""

from pyrr import Vector3

from revolve2.ci_group import terrains
from revolve2.ci_group.interactive_objects import Ball
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.v2 import ActiveHingeV2, BodyV2, BrickV2
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulation.scene import Pose
from revolve2.simulators.mujoco_simulator import LocalSimulator


def make_body() -> BodyV2:
    """
    Create a body for the robot.

    :returns: The created body.
    """
    # A modular robot body follows a 'tree' structure.
    # The 'Body' class automatically creates a center 'core'.
    # From here, other modular can be attached.
    # Modules can be attached in a rotated fashion.
    # This can be any angle, although the original design takes into account only multiples of 90 degrees.
    body = BodyV2()
    body.core_v2.left_face.bottom = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.left_face.bottom.attachment = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.left_face.bottom.attachment.attachment = BrickV2(RightAngles.DEG_0)
    body.core_v2.right_face.bottom = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.right_face.bottom.attachment = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.right_face.bottom.attachment.attachment = BrickV2(RightAngles.DEG_0)
    return body


def main() -> None:
    """Run the simulation."""
    # Set up logging to give output of your simulation into the command line interface (CLI).
    setup_logging()

    # Set up a random number generator, used later for the brain.
    rng = make_rng_time_seed()

    # Create a body for the robot.
    body = make_body()

    """
    Here we create a brain for the robot.
    We choose a 'CPG' brain with random parameters.
    If you want to know more about CPGs checkout the Methods section in: https://doi.org/10.1038/s41598-023-48338-4. 
    """
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)

    """Once we have a body and a brain we combine it into a ModularRobot."""
    robot = ModularRobot(body, brain)

    """
    To simulate our newly created robot, we create a modular robot scene.
    This scene is a combination of one or more modular robots positioned in a given terrain.
    """
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Additionally to robots you can also add interactive objects to the scene.
    scene.add_interactive_object(
        Ball(radius=0.1, mass=0.1, pose=Pose(Vector3([-0.5, 0.5, 0])))
    )

    """
    After we have the scene ready we create a simulator that will perform the simulation.
    This tutorial chooses to use Mujoco, but your version of revolve might contain other simulators as well.
    
    For mujoco we can select either the `native` mujoco viewer (more performance) or our `custom` viewer (which is more flexible for adjustments).
    """
    simulator = LocalSimulator(viewer_type="native")

    # `batch_parameters` are important parameters for simulation.
    # Here, we use the parameters that are standard in CI Group.
    batch_parameters = make_standard_batch_parameters()
    batch_parameters.simulation_time = 60  # Here we update our simulation time.

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
