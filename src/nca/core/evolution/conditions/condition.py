from abc import abstractmethod

from nca.core.actor.actors import Actors
from nca.core.ecology.population import Population


class Condition:

    def __init__(self, condition_value, value_type: type):
        self.condition_value: value_type = condition_value

    def terminate(self, population: Population):
        agents = self.filter(population)
        if len(agents) > 0:
            return True
        return False

    @abstractmethod
    def filter(self, population: Population) -> Actors:
        pass

    @abstractmethod
    def __str__(self):
        pass


class FitnessCondition(Condition):

    def __init__(self, fitness_value: float):
        super().__init__(fitness_value, float)

    def filter(self, population: Population):
        filtered_individuals = Actors()
        for individual in population.individuals:
            if individual.fitness.value() >= self.condition_value:
                filtered_individuals.append(individual)
        return filtered_individuals

    def __str__(self):
        return "FitnessCondition (" + str(self.condition_value) + " fitness value)"


class EvaluationsCondition(Condition):

    def __init__(self, number_of_evaluations: int):
        super().__init__(number_of_evaluations, int)

    def filter(self, population: Population):
        if population.age.generations >= self.condition_value:
            return population.individuals

        return Actors()

    def __str__(self):
        return "EvaluationCondition (" + str(self.condition_value) + " generations)"


class ImprovementCondition(Condition):

    def __init__(self, no_improvement_count: int):
        super().__init__(no_improvement_count, int)

    def filter(self, population: Population):
        if population.age.no_improvement_count >= self.condition_value:
            return population.individuals

        return Actors()

    def __str__(self):
        return "ImprovementCondition (" + str(self.condition_value) + " generations no improvements)"
