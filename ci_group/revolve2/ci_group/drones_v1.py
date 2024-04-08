import numpy as np

from revolve2.modular_robot.body.drone_v1 import DroneCoreV1, MotorV1, DroneBodyV1
from pyrr import Quaternion, Vector3


def all() -> list[DroneBodyV1]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [cross_v1()]

def cross_v1() -> DroneBodyV1:
    """
    Get the cross drone.

    :returns: the robot.
    """
    body = DroneBodyV1()

    q = Quaternion.from_x_rotation(np.pi / 3.0)
    body.core_v1.add_attachment(MotorV1(position=Vector3([0.5, 0.75, 0.0]), orientation=q))
    body.core_v1.add_attachment(MotorV1(position=Vector3([0.5, -0.5, 0.0]), orientation=Quaternion()))
    body.core_v1.add_attachment(MotorV1(position=Vector3([-0.5, -0.5, 0.0]), orientation=Quaternion()))
    body.core_v1.add_attachment(MotorV1(position=Vector3([-0.5, 0.5, 0.0]), orientation=Quaternion()))

    return body