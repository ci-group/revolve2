"""Contains various drone modular robots."""

import numpy as np
import numpy.typing as npt
from pyrr import Quaternion, Vector3

from revolve2.modular_robot.body.drone import DroneBodyImpl, MotorImpl


def all() -> list[DroneBodyImpl]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [random(), cross()]


def random() -> DroneBodyImpl:
    """
    Random drone, randomizing number of motors/arms and where they are positioned and orientated in relation to the core.

    :returns: the robot.
    """
    body = DroneBodyImpl()

    num_arms = np.random.randint(0, 16)

    def random_quaternion() -> Quaternion:
        """
        Calculate a random quaternion with a uniform distribution.

        :returns: a random quaternion
        """
        u, v, w = np.random.uniform(0, 1, 3)
        q = [
            np.sqrt(1 - u) * np.sin(np.pi * v),
            np.sqrt(1 - u) * np.cos(np.pi * v),
            np.sqrt(u) * np.sin(np.pi * w),
            np.sqrt(u) * np.cos(np.pi * w),
        ]
        return Quaternion(q)

    def random_position() -> npt.NDArray[np.float64]:
        """Calculate a random 3D position in a sphere with a radius of 1.

        :returns: A numpy array with 3 elements, uniformly random in a sphere of 1 radius.
        """
        pos = np.random.uniform(-1, 1, 3)
        while np.linalg.norm(pos) > 1:
            pos = np.random.uniform(-1, 1, 3)
        return pos

    for _ in range(num_arms):
        clockwise_rotation = bool(np.random.randint(2))
        body.core.add_attachment(
            MotorImpl(
                position=Vector3(random_position()),
                orientation=random_quaternion(),
                clockwise_rotation=clockwise_rotation,
            )
        )

    return body


def cross() -> DroneBodyImpl:
    """
    Get the cross drone.

    0     0
       _
      |_|
    0     0

    :returns: The robot.
    """
    body = DroneBodyImpl()

    body.core.add_attachment(
        MotorImpl(
            position=Vector3([-0.1, 0.1, 0]),
            orientation=Quaternion(),
            clockwise_rotation=True,
        )
    )
    body.core.add_attachment(
        MotorImpl(
            position=Vector3([0.1, 0.1, 0]),
            orientation=Quaternion(),
            clockwise_rotation=False,
        )
    )
    body.core.add_attachment(
        MotorImpl(
            position=Vector3([-0.1, -0.1, 0]),
            orientation=Quaternion(),
            clockwise_rotation=True,
        )
    )
    body.core.add_attachment(
        MotorImpl(
            position=Vector3([0.1, -0.1, 0]),
            orientation=Quaternion(),
            clockwise_rotation=False,
        )
    )

    return body


def plus() -> DroneBodyImpl:
    """
    Get the plus drone.

        0
        _
    0  |_|  0

        0

    :returns: the robot.
    """
    body = DroneBodyImpl()

    body.core.add_attachment(
        MotorImpl(
            position=Vector3([0.0, 0.1, 0]),
            orientation=Quaternion(),
            clockwise_rotation=False,
        )
    )
    body.core.add_attachment(
        MotorImpl(
            position=Vector3([0.1, 0.0, 0]),
            orientation=Quaternion(),
            clockwise_rotation=True,
        )
    )
    body.core.add_attachment(
        MotorImpl(
            position=Vector3([0.0, -0.1, 0]),
            orientation=Quaternion(),
            clockwise_rotation=False,
        )
    )
    body.core.add_attachment(
        MotorImpl(
            position=Vector3([-0.1, 0.0, 0]),
            orientation=Quaternion(),
            clockwise_rotation=True,
        )
    )

    return body
