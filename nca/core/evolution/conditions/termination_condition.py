from abc import abstractmethod

from nca.core.ecology.population import Population


class TerminationCondition:

    def __init__(self, condition_value):
        self.condition_value: object = condition_value

    @abstractmethod
    def terminate(self, population: Population) -> bool:
        pass


class FitnessCondition(TerminationCondition):

    def __init__(self, fitness_value: float):
        super().__init__(fitness_value)

    def terminate(self, population: Population):
        for individual in population.individuals:
            if individual.fitness >= self.condition_value:
                return True
        return False


class EvaluationsCondition(TerminationCondition):

    def __init__(self, number_of_evaluations: int):
        super().__init__(number_of_evaluations)

    def terminate(self, population: Population):
        if population.age.generations >= self.condition_value:
            return True
        return False


class ImprovementCondition(TerminationCondition):

    def __init__(self, no_improvement_count: int):
        super().__init__(no_improvement_count)

    def terminate(self, population: Population):
        if population.age.no_improvement_count >= self.condition_value:
            return True
        return False
