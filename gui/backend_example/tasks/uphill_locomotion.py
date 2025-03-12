"""Fitness functions for uphill locomotion."""

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))

from revolve2.modular_robot_simulation import ModularRobotSimulationState


def z_displacement(
    begin_state: ModularRobotSimulationState, end_state: ModularRobotSimulationState
) -> float:
    """
    Calculate the distance traveled on the z-axis by a single modular robot.

    :param begin_state: Begin state of the robot.
    :param end_state: End state of the robot.
    :returns: The calculated fitness.
    """
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position

    return -(begin_position.z - end_position.z)

