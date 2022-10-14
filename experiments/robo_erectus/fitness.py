from revolve2.core.physics.running._results import EnvironmentResults
import measures


def calculate(self, environment_results) -> float:
    return function_types[self._fitness_function](environment_results)


@staticmethod
def displacement_height_groundcontact(environment_results: EnvironmentResults) -> float:
    return (
        measures.ground_contact_measure(environment_results)
        * measures.displacement_measure(environment_results)
        / measures.max_height_relative_to_avg_height_measure(environment_results)
    )


@staticmethod
def displacement_only(environment_results: EnvironmentResults) -> float:
    return measures.displacement_measure(environment_results)


function_types = {
    "displacement_height_groundcontact": displacement_height_groundcontact,
    "displacement_only": displacement_only,
}
