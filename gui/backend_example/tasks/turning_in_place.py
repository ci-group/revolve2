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
    
    fitness = 1 / (1 + average_distance) + (trajectory_length) * 0.5 # add distance travelled, so we don't get a block that doesn't move

    return fitness