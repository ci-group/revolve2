import random
from typing import List

import numpy as np

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.algorithm.selection.selection import ParentSelection


class RandomParentSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: List[Individual]) -> Individual:
        return random.choice(individuals)


class TournamentSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: List[Individual]) -> Individual:
        return max(np.random.choice(individuals, self.configuration.tournament_size), key=lambda x: x.fitness)


class RouletteWheelSelection(ParentSelection):

    def algorithm(self, individuals: List[Individual]) -> Individual:
        #TODO simplify
        return random.choices(individuals, weights=[agent.fitness.fitness for agent in individuals])[0]


class NullParentSelection(RandomParentSelection):
    pass
