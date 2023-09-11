from typing import List, Tuple

from revolve2.core.physics.running import BatchResults

from ._body import Body
from ._body_state import BodyState


def get_body_states_multiple_isolated_robots(
    bodies: List[Body], batch_results: BatchResults
) -> List[Tuple[BodyState, BodyState]]:
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
