"""Classes and functions to describe and work with modular robots as used in the CI Group at VU Amsterdam."""

from ._modular_robot import ModularRobot
from ._modular_robot_control_interface import ModularRobotControlInterface

__all__ = [
    "ModularRobot",
    "ModularRobotControlInterface",
]
