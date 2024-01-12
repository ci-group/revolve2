from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.dummy import BrainDummy

from .._config import Config
from .._uuid_key import UUIDKey
from ._remote import run_remote


def test_physical_robot(
    robot: ModularRobot | Body,
    hostname: str,
    hinge_mapping: dict[UUIDKey[ActiveHinge], int],
    inverse_servos: dict[int, bool],
) -> None:
    """
    Remotely connect to a physical robot and provide manual controls to test it.

    :param robot: Body of the robot.
    :param hostname: Hostname of the robot.
    :param hinge_mapping: map each active hinge object to a specific Servo with its ID (int).
    :param inverse_servos: If a servo is mounted in the wrong direction on the body one can fix it by inversing the action. inverse_servos allows you to inverse specific servos with their gpio number as key.
    """
    if isinstance(robot, Body):
        body = robot
        brain = BrainDummy()
        robot = ModularRobot(body=body, brain=brain)
    config = Config(
        modular_robot=robot,
        hinge_mapping=hinge_mapping,
        run_duration=99999,
        control_frequency=20,
        initial_hinge_positions={
            UUIDKey(active_hinge.value): 0.0 for active_hinge in hinge_mapping.keys()
        },
        inverse_servos=inverse_servos,
    )

    run_remote(
        config=config, hostname=hostname, on_prepared=lambda: None, manual_mode=True
    )
