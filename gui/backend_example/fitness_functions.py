"""Standard fitness functions for modular robots."""

import math
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from revolve2.modular_robot_simulation import ModularRobotSimulationState
from revolve2.gui.backend_example.config_simulation_parameters import STANDARD_SIMULATION_TIME


def xy_displacement(
    begin_state: ModularRobotSimulationState, end_state: ModularRobotSimulationState
) -> float:
    """
    Calculate the distance traveled on the xy-plane by a single modular robot.

    :param begin_state: Begin state of the robot.
    :param end_state: End state of the robot.
    :returns: The calculated fitness.
    """
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position
    return math.sqrt(
        (begin_position.x - end_position.x) ** 2
        + (begin_position.y - end_position.y) ** 2
    )

    
def x_speed_Miras2021(begin_state: ModularRobotSimulationState, end_state: ModularRobotSimulationState, simulation_time=STANDARD_SIMULATION_TIME) -> float:
    """Goal:
        Calculate the fitness for speed in x direction for a single modular robot according to 
            Miras (2021).
    -------------------------------------------------------------------------------------------
    Input:
        begin_state: The starting state of the robot.
        end_state: Final state reached by the robot.
        
        derived inputs->
        x_distance: The distance traveled in the x direction.
        simulation_time: The time of the simulation.
    -------------------------------------------------------------------------------------------
    Output:
        The calculated fitness.
    """
    # Begin and end Position
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position

    x_distance = abs(end_position.x - begin_position.x)

    # Calculate the speed in x direction
    vx = float((x_distance / simulation_time) * 100)
    if vx > 0:
        return vx
    elif vx == 0:
        return -0.1
    else:
        return vx / 10


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
        math.sqrt(abs((state.get_pose().position.x - center.x) ** 2 + (state.get_pose().position.y - center.y) ** 2  - radius ** 2))
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


# def x_efficiency(xbest: float, eexp: float, simulation_time: float) -> float:
#     """Goal:
#         Calculate the efficiency of a robot for locomotion in x direction.
#     -------------------------------------------------------------------------------------------
#     Input:
#         xbest: The furthest distance traveled in the x direction.
#         eexp: The energy expended.
#         simulation_time: The time of the simulation.
#     -------------------------------------------------------------------------------------------
#     Output:
#         The calculated fitness.
#     """
#     def food(xbest, bmet):
#         # Get food
#         if xbest <= 0:
#             return 0
#         else:
#             food = (xbest / 0.05) * (80 * bmet)
#             return food
    
#     def scale_EEXP(eexp, bmet):
#         return eexp / 346 * (80 * bmet)
    
#     # Get baseline metabolism
#     bmet = 80
#     battery = -bmet * simulation_time + food(xbest, bmet) - scale_EEXP(eexp, bmet)
    
#     return battery