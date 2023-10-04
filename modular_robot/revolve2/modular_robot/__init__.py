"""Classes and functions to describe and work with modular robots as used in the CI Group at VU Amsterdam."""

from ._modular_robot import ModularRobot
from ._modular_robot_control_interface import ModularRobotControlInterface
from ._modular_robot_sensor_state import ModularRobotSensorState

__all__ = [
    "ModularRobot",
    "ModularRobotControlInterface",
    "ModularRobotSensorState",
]
