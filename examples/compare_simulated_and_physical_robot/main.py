from revolve2.ci_group.modular_robots_v2 import gecko_v2
from revolve2.ci_group import terrains
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_physical import UUIDKey
from revolve2.modular_robot_simulation import ModularRobotScene

from simulate_scene_iteratively import simulate_scene_iteratively


def main():
    rng = make_rng_time_seed()  # Make a rng generator for the brain.

    """Here we initialize the Modular Robot."""
    body = gecko_v2()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body=body, brain=brain)

    """Here we prepare the config for the physical twin of our Robot."""
    hinges = body.find_modules_of_type(ActiveHinge)
    pins = [6, 12, 13, 16, 17, 21]
    hinge_mapping = {UUIDKey(hinge): pin for hinge, pin in zip(hinges, pins)}
    # TODO: config

    """Now we create a scene with our simulated robot, to validate our physical robot."""
    mr_scene = ModularRobotScene(terrain=terrains.flat())
    mr_scene.add_robot(robot)

    scene, _ = mr_scene.to_simulation_scene()
    simulate_scene_iteratively(scene)

    
if __name__ == '__main__':
    main()
