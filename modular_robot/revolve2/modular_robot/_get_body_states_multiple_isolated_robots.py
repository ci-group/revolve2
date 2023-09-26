from typing import Sequence

from revolve2.modular_robot._body import Body
from revolve2.modular_robot._body_state import BodyState
from revolve2.simulation.running import BatchResults


def get_body_states_multiple_isolated_robots(
    bodies: Sequence[Body], batch_results: BatchResults
) -> list[tuple[BodyState, BodyState]]:
    """
    Get the first and last body state of a robot from a simulation simulating only a single robot.

    :param bodies: The bodies of the robots.
    :param batch_results: The simulation results.
    :returns: The first and last body state for each robot body.
    """
    return [
        (
            body.body_state_from_actor_state(
                environment_results.environment_states[0].actor_states[0]
            ),
            body.body_state_from_actor_state(
                environment_results.environment_states[-1].actor_states[0]
            ),
        )
        for body, environment_results in zip(bodies, batch_results.environment_results)
    ]
