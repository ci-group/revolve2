import random
from typing import List

import numpy as np

from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.evolution.selection.selection import ParentSelection


class RandomParentSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors) -> List[Individual]:
        return [individual for individual in random.sample(individuals, k=self.configuration.number_of_parents)]


class TournamentSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors) -> List[Individual]:
        return [max(np.random.choice(individuals, self.configuration.tournament_size),
                    key=lambda x: x.get_fitness())
                for _ in range(self.configuration.number_of_parents)]


class RouletteWheelSelection(ParentSelection):

    def algorithm(self, individuals: Actors) -> List[Individual]:
        individual_weights = [abs(agent.get_fitness()) for agent in individuals]
        return [individual for individual in random.choices(individuals, k=self.configuration.number_of_parents, weights=individual_weights)]


class NullParentSelection(RandomParentSelection):
    pass
