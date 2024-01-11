"""Simulate the modular robot for comparison with the physical counterpart."""
from revolve2.ci_group import terrains
from revolve2.ci_group.modular_robots_v2 import gecko_v2
from revolve2.ci_group.simulation_parameters import make_standard_batch_parameters
from revolve2.modular_robot_simulation import test_robot
from revolve2.simulators.mujoco_simulator import LocalSimulator


def main() -> None:
    """Run the simulation part of the example."""
    """
    Here we define the Modular Robot we want to test.
    You can use either a ModularRobot instance or a Body instance.
    """
    body = gecko_v2()

    """Now we test our simulated robot, to validate our physical robot."""
    simulator = LocalSimulator(manual_control=True)
    batch_parameters = make_standard_batch_parameters()
    test_robot(
        robot=body,
        terrain=terrains.flat(),
        simulator=simulator,
        batch_parameters=batch_parameters,
    )


if __name__ == "__main__":
    main()
