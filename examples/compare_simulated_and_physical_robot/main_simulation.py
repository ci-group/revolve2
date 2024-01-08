"""Simulate the modular robot for comparison with the physical counterpart."""
from revolve2.ci_group import terrains
from revolve2.ci_group.modular_robots_v2 import gecko_v2
from revolve2.modular_robot_simulation import test_robot


def main() -> None:
    """Run the simulation part of the example."""
    """
    Here we define the Modular Robot we want to test.
    You can use either a ModularRobot instance or a Body instance.
    """
    body = gecko_v2()

    """Now we test our simulated robot, to validate our physical robot."""
    test_robot(robot=body, terrain=terrains.flat())


if __name__ == "__main__":
    main()
