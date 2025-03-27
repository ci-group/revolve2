"""Fitness functions for uphill locomotion."""

import os
import sys
import numpy as np
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


def uphill_displacement(
    begin_state: ModularRobotSimulationState,
    end_state: ModularRobotSimulationState,
    tilt_angle: float,
    tilt_direction: tuple[float, float],
) -> float:
    """
    Compute the displacement in the uphill direction.

    Parameters:
    -----------
    begin_state : ModularRobotSimulationState
        Initial state of the robot.
    end_state : ModularRobotSimulationState
        Final state of the robot.
    tilt_angle : float
        The tilt angle in degrees.
    tilt_direction : tuple[float, float]
        The (x, y) direction of the tilt.

    Returns:
    --------
    float
        The displacement along the uphill direction.
    """
    # Convert tilt angle to radians
    tilt_angle_rad = np.radians(tilt_angle)

    # Normalize tilt direction
    tilt_dir = np.array(tilt_direction)
    tilt_dir = tilt_dir / np.linalg.norm(tilt_dir)

    # Compute the uphill direction (opposite to tilt)
    uphill_direction = np.array([-tilt_dir[0] * np.sin(tilt_angle_rad),
                                 -tilt_dir[1] * np.sin(tilt_angle_rad),
                                 np.cos(tilt_angle_rad)])
    
    # Get robot center of mass positions
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position

    # Compute displacement vector
    displacement = np.array([end_position.x - begin_position.x, 
                             end_position.y - begin_position.y, 
                             end_position.z - begin_position.z])

    # Project displacement onto the uphill direction
    uphill_displacement = np.dot(displacement, uphill_direction)

    return uphill_displacement