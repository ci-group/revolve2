"""An example on how to make a config file for running a physical modular robot."""
import pickle

from revolve2.ci_group.modular_robots_v1 import ant_v1
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_physical import Config, UUIDKey


def main() -> None:
    """Create a Config for the physical robot."""
    rng = make_rng_time_seed()
    # Create a modular robot, similar to what was done in the simulate_single_robot example. Of course, you can replace this with your own robot, such as one you have optimized using an evolutionary algorithm.
    body = ant_v1()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body, brain)

    """
    A configuration consists of the follow parameters:
    - modular_robot: The ModularRobot object, exactly as you would use it in simulation.
    - hinge_mapping: This maps active hinges to GPIO pins on the physical modular robot core. TODO in the next version we will add a guide to the documentation on how to create a modular robot body. For now, ask the CI Group lab.
    - run_duration: How long to run the robot for in seconds.
    - control_frequency: Frequency at which to call the brain control functions in seconds. If you also ran the robot in simulation, this must match your setting there.
    - initial_hinge_positions: Initial positions for the active hinges. In Revolve2 the simulator defaults to 0.0.
    - inverse_servos: Sometimes servos on the physical robot are mounted backwards by accident. Here you inverse specific servos in software. Example: {13: True} would inverse the servo connected to GPIO pin 13.
    """
    active_hinges = body.find_modules_of_type(ActiveHinge)

    """These are examples for gpio pins as indicated on the robots HAT. For your own robot, adjust these ids. Make sure that you have the same amount of pins as hinges."""
    gpio_pins = [6, 12, 13, 16, 19, 20, 26, 21]

    assert len(active_hinges) == len(
        gpio_pins
    ), f"ERROR: there are {len(active_hinges)} but {len(gpio_pins)} pins."

    config = Config(
        modular_robot=robot,
        hinge_mapping={
            UUIDKey(active_hinge): gpio_pin
            for active_hinge, gpio_pin in zip(active_hinges, gpio_pins)
        },
        run_duration=10,
        control_frequency=10,
        initial_hinge_positions={
            UUIDKey(active_hinge): 0.0 for active_hinge in active_hinges
        },
        inverse_servos={},
    )

    # Serialize the configuration object and save it to a file. This file will later be read by the modular robot core.
    with open("config.pickle", "wb") as f:
        pickle.dump(config, f)


if __name__ == "__main__":
    main()
