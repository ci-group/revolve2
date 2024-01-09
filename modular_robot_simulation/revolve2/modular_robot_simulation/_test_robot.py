from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import Body
from revolve2.modular_robot.brain.dummy import BrainDummy
from revolve2.simulation.simulator import BatchParameters, Simulator

from ._modular_robot_scene import ModularRobotScene
from ._simulate_scenes import simulate_scenes
from ._terrain import Terrain


def test_robot(
    robot: ModularRobot | Body,
    terrain: Terrain,
    simulator: Simulator,
    batch_parameters: BatchParameters,
) -> None:
    """
    Test a robot with a manual brain.

    :param robot: The ModularRobot or Body instance.
    :param terrain: The terrain to test on.
    :param simulator: The simulator.
    :param batch_parameters: The batch parameters.
    """
    if isinstance(robot, Body):
        body = robot
        brain = BrainDummy()
        robot = ModularRobot(body=body, brain=brain)

    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)

    simulate_scenes(
        simulator=simulator, batch_parameters=batch_parameters, scenes=scene
    )
