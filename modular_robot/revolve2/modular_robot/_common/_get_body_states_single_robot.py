from revolve2.simulation.running import EnvironmentResults

from ._body import Body
from ._body_state import BodyState


def get_body_states_single_robot(
    body: Body, environment_results: EnvironmentResults
) -> tuple[BodyState, BodyState]:
    """
    Get the first and last body state of a robot from a simulation simulating only a single robot.

    :param body: The body of the robot.
    :param environment_results: The simulation result.
    :returns: The first and last body state.
    """
    return (
        body.body_state_from_actor_state(
            environment_results.environment_states[0].actor_states[0]
        ),
        body.body_state_from_actor_state(
            environment_results.environment_states[-1].actor_states[0]
        ),
    )
