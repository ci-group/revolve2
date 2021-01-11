import random
from typing import List

from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.evolution.selection.selection import SurvivorSelection


class FitnessSteadyStateSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: List[Individual]) -> List[Individual]:
        return sorted(individuals, key=lambda x: x.get_fitness(), reverse=True)


class GenerationalSteadyStateSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: List[Individual]) -> List[Individual]:
        return sorted(individuals, key=lambda x: x.age.generations, reverse=False)


class ElitismSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: List[Individual]) -> List[Individual]:
        sorted_individuals: List[Individual] = sorted(individuals, key=lambda x: x.get_fitness(), reverse=True)
        elite_individuals: List[Individual] = sorted_individuals[:self.configuration.selection_size]
        non_elite_individuals: List[Individual] = sorted_individuals[self.configuration.selection_size:]
        new_individuals = elite_individuals
        random.shuffle(non_elite_individuals)
        new_individuals.extend(non_elite_individuals)
        return new_individuals


class RoundRobinTournament(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors) -> List[Individual]:
        new_individuals = []
        for _ in range(self.configuration.population_size + self.configuration.selection_size):
            tournament_individuals = random.choices(individuals, k=2) # TODO parameterize
            new_individuals.append(max(tournament_individuals, key=lambda x: x.get_fitness()))
        return new_individuals


class RandomSurvivorSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors) -> List[Individual]:
        random.shuffle(individuals)
        return individuals


class MuLambdaSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors) -> List[Individual]:
        raise Exception("Unimplemented Mu Lambda Selection")


class NullSurvivorSelection(FitnessSteadyStateSelection):
    pass
