"""Simulate the modular robot for comparison with the physical counterpart."""
from revolve2.ci_group import terrains
from revolve2.ci_group.modular_robots_v2 import gecko_v2
from revolve2.ci_group.simulation import make_standard_batch_parameters
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator


def main() -> None:
    """Run the checking."""
    rng = make_rng_time_seed()  # Make a rng generator for the brain.

    """Here we initialize the Modular Robot."""
    body = gecko_v2()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body=body, brain=brain)

    """Now we create a scene with our simulated robot, to validate our physical robot."""
    simulator = LocalSimulator(manual_control=True)
    batch_parameters = make_standard_batch_parameters()

    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    simulate_scenes(
        simulator=simulator, batch_parameters=batch_parameters, scenes=scene
    )


if __name__ == "__main__":
    main()
