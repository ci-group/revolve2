"""An example on how to remote control a physical modular robot."""

import pickle

from revolve2.modular_robot_physical.remote import run_remote


def on_prepared() -> None:
    """Do things when the robot is prepared and ready to start the controller."""
    print("Done. Press enter to start the brain.")
    input()


def main() -> None:
    with open("conf.pickle", "rb") as f:
        config = pickle.load(f)

    print("Initializing robot..")
    run_remote(
        config=config,
        hostname="localhost",
        debug=True,
        on_prepared=on_prepared,
    )


if __name__ == "__main__":
    main()
