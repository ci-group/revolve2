"""Standard fitness functions for modular robots."""

import math

from revolve2.modular_robot_simulation.simulation_states import (
    MultiBodySystemSimulationState,
)
from scipy.spatial.transform import Rotation


def xy_displacement(
    begin_state: MultiBodySystemSimulationState,
    end_state: MultiBodySystemSimulationState,
) -> float:
    """
    Calculate the distance traveled on the xy-plane by a single multi-body-system.

    :param begin_state: Begin state of the robot or object.
    :param end_state: End state of the robot or object.
    :returns: The calculated fitness.
    """
    begin_position = begin_state.get_pose().position
    end_position = end_state.get_pose().position
    return math.sqrt(
        (begin_position.x - end_position.x) ** 2
        + (begin_position.y - end_position.y) ** 2
    )


def total_rotation(
    states: list[MultiBodySystemSimulationState], clockwise: bool = True, axis: int = 2
) -> float:
    """
    Calculate the total rotation (in radiant) of the multi-body-system during the simulation.

    :param states: List of states in the simulation.
    :param clockwise: Whether to count rotation clockwise.
    :param axis: Axis of the rotation (x=0,y=1,z=2).
    :returns: The calculated fitness.
    """
    angle = 0
    for i in range(len(states) - 1):
        *axi, wi = states[i].get_pose().orientation
        *axe, we = states[i + 1].get_pose().orientation
        rot_i, rot_e = Rotation.from_quat([wi, *axi]), Rotation.from_quat([we, *axe])

        angle_i, angle_e = rot_i.as_euler("xyz"), rot_e.as_euler("xyz")
        angle += (
            -angle_e[axis] + angle_i[axis]
            if clockwise
            else angle_e[axis] - angle_i[axis]
        )
    return angle
