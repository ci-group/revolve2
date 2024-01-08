from revolve2.ci_group.simulation import make_standard_batch_parameters
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import Body
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene
from revolve2.simulation.simulator import Simulator

from ._simulate_scenes import simulate_scenes
from ._terrain import Terrain


def test_robot(
    robot: ModularRobot | Body, terrain: Terrain, simulator: Simulator
) -> None:
    """
    Test a robot with a manual brain.

    :param robot: The ModularRobot or Body instance.
    :param terrain: The terrain to test on.
    :param simulator: The simulator.
    """
    if isinstance(robot, Body):
        rng = make_rng_time_seed()
        body = robot
        brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
        robot = ModularRobot(body=body, brain=brain)

    batch_parameters = make_standard_batch_parameters()

    scene = ModularRobotScene(terrain=terrain)
    scene.add_robot(robot)

    simulate_scenes(
        simulator=simulator, batch_parameters=batch_parameters, scenes=scene
    )
