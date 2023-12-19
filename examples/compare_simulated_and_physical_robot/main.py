"""Compare physical robot with simulation behavior to validate the build."""
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

    """Here we prepare the config for the physical twin of our Robot."""
    # hinges = body.find_modules_of_type(ActiveHinge)
    # pins = [6, 12, 13, 16, 17, 21]
    # hinge_mapping = {UUIDKey(hinge): pin for hinge, pin in zip(hinges, pins)}
    # TODO: make the physical counterpart

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
