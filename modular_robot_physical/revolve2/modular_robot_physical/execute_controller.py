"""Execute the controller on a robot."""
from ._physical_robot_controller import PhysicalRobotController


def main() -> None:
    """Run the script."""
    PhysicalRobotController().main()


if __name__ == "__main__":
    main()
