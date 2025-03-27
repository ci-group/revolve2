"""Fitness functions for turning in place."""

import math
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))

from revolve2.modular_robot_simulation import ModularRobotSimulationState


def circular_trajectory(simulation_states: list[ModularRobotSimulationState], radius: float) -> float:
    """Goal:
        Calculate the fitness for a circular trajectory for a single modular robot.
    -------------------------------------------------------------------------------------------
    Input:
        simulation_states: The states of the robot during the simulation.
        radius: The radius of the circle.
    -------------------------------------------------------------------------------------------
    Output:
        The calculated fitness.
    """
    
    # Get the center of the circle
    center = simulation_states[0].get_pose().position
    center.x += radius

    # Calculate the distance from the center
    distances = [
        math.sqrt(
            abs((state.get_pose().position.x - center.x) ** 2
              + (state.get_pose().position.y - center.y) ** 2
                  - radius ** 2)
                )
        for state in simulation_states
    ]

    # Calculate the average distance from the center of circle
    average_distance = sum(distances) / len(distances)

    # calculate distance travelled
    trajectory_length = 0
    for i in range(1, len(simulation_states)):
        trajectory_length += math.sqrt(
            (simulation_states[i].get_pose().position.x - simulation_states[i - 1].get_pose().position.x) ** 2
            + (simulation_states[i].get_pose().position.y - simulation_states[i - 1].get_pose().position.y) ** 2
        )
    
    fitness = 1 / (1 + average_distance) + (trajectory_length) * 0.1 # add distance travelled, so we don't get a block that doesn't move

    return fitness

def turn_360_fitness(
    simulation_states: list[ModularRobotSimulationState], 
    expected_radius: float = 0.1  # Small radius to encourage in-place turning
) -> float:
    """
    Fitness function to maximize a robot's precise 360-degree rotation around a fixed point.
    
    Parameters:
    -----------
    simulation_states : list[ModularRobotSimulationState]
        The states of the robot during the simulation.
    expected_radius : float, optional
        The expected radius of rotation (default is 0.1 to encourage tight turning)
    
    Returns:
    --------
    float
        The calculated fitness score for the rotation.
    """
    if len(simulation_states) < 2:
        return -10  # Penalize insufficient movement
    
    # Initial position as the center of rotation
    center_x = simulation_states[0].get_pose().position.x
    center_y = simulation_states[0].get_pose().position.y
    
    # Track rotation metrics
    total_angle_covered = 0
    radial_deviations = []
    
    # Compute angular progression and radial consistency
    previous_angle = None
    for state in simulation_states[1:]:
        # Current position
        curr_x = state.get_pose().position.x
        curr_y = state.get_pose().position.y
        
        # Calculate distance from center
        radial_distance = math.sqrt(
            (curr_x - center_x)**2 + 
            (curr_y - center_y)**2
        )
        
        # Track radial deviation
        radial_deviations.append(abs(radial_distance - expected_radius))
        
        # Calculate current angle from center
        current_angle = math.atan2(curr_y - center_y, curr_x - center_x)
        
        # Handle angle wrapping and progression
        if previous_angle is not None:
            angle_diff = current_angle - previous_angle
            
            # Normalize angle difference
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            total_angle_covered += abs(angle_diff)
        
        previous_angle = current_angle
    
    # Compute fitness components
    # 1. Rotation completeness
    rotation_completeness = total_angle_covered / (2 * math.pi)
    
    # 2. Radial consistency
    avg_radial_deviation = sum(radial_deviations) / len(radial_deviations)
    radial_penalty = avg_radial_deviation * 10
    
    # 3. Penalize incomplete or over-rotation
    rotation_penalty = abs(1 - rotation_completeness) * 5
    
    # Combine fitness components
    fitness = (
        10 * rotation_completeness  # Reward for completing rotation
        - radial_penalty  # Penalize radial deviation
        - rotation_penalty  # Penalize rotation inaccuracy
    )
    
    return max(fitness, -10)
    