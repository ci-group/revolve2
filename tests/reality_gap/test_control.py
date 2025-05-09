"""An example on how to remote control a physical gecko robot."""

import socket
import time
import logging
from pyrr import Vector3

from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_physical import Config, UUIDKey
from revolve2.modular_robot_physical.remote import run_remote
from revolve2.standards.modular_robots_v2 import gecko_v2
from revolve2.experimentation.logging import setup_logging


def check_connection(host: str, port: int = 20812, timeout: int = 2) -> bool:
    """Check if we can connect to the robot."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception as e:
        print(f"Connection check failed: {e}")
        return False

def on_prepared() -> None:
    """Do things when the robot is prepared and ready to start the controller."""
    print("Done. Press enter to start the brain.")
    input()


def main() -> None:
    """Remote control a physical gecko robot."""
    # Setup logging
    setup_logging()
    logging.getLogger().setLevel(logging.DEBUG)
    
    # Connection settings
    ROBOT_IP = "10.15.3.39"
    
    # print(f"Checking connection to robot at {ROBOT_IP}...")
    # if not check_connection(ROBOT_IP):
    #     print("Could not connect to robot")
    #     return
    # print("Successfully connected to robot")

    # Create a minimal test configuration first
    body = gecko_v2()
    active_hinges = body.find_modules_of_type(ActiveHinge)
    print(f"Found {len(active_hinges)} active hinges in the gecko_v2 body")
    
    # Start with just one hinge for testing
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=make_rng_time_seed())
    robot = ModularRobot(body, brain)

    # Simplified hinge mapping for initial test
    hinge_mapping = {
        UUIDKey(active_hinges[0]): 0,
        UUIDKey(active_hinges[1]): 1,
        UUIDKey(active_hinges[2]): 2,
        UUIDKey(active_hinges[3]): 13,
        UUIDKey(active_hinges[4]): 14,
        UUIDKey(active_hinges[5]): 15,
    }

    config = Config(
        modular_robot=robot,
        hinge_mapping=hinge_mapping,
        run_duration=10,  # Shorter duration for testing
        control_frequency=20,
        initial_hinge_positions={UUIDKey(active_hinge): 0.0 for active_hinge in active_hinges},
        inverse_servos={},
    )

    print("Initializing robot..")
    try:
        run_remote(
            config=config,
            hostname=ROBOT_IP,
            debug=True,
            on_prepared=on_prepared,
            display_camera_view=False,
        )
    except Exception as e:
        print(f"Error during robot control: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
