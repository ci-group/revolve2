"""Classes and functions to describe and work with modular robots as used in the CI Group at VU Amsterdam."""

from ._aerial_robot import AerialRobot
from ._aerial_robot_control_interface import AerialRobotControlInterface

__all__ = [
    "AerialRobot",
    "AerialRobotControlInterface",
]
