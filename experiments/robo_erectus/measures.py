import math
from revolve2.core.physics.running._results import EnvironmentResults


def displacement_measure(environment_results: EnvironmentResults) -> float:
    begin_state = environment_results.environment_states[0].actor_states[0]
    end_state = environment_results.environment_states[-1].actor_states[0]
    distance = math.sqrt(
        (begin_state.position[0] - end_state.position[0]) ** 2
        + ((begin_state.position[1] - end_state.position[1]) ** 2)
    )
    return float(distance)


def max_height_relative_to_avg_height_measure(environment_results: EnvironmentResults) -> float:
    heights = [
        environment_results.environment_states[i + 1].actor_states[0].position[2]
        for i in range(len(environment_results.environment_states) - 1)
    ]
    return float(max(heights) / (sum(heights) / len(heights)))
