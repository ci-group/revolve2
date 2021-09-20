import string
from typing import List

import numpy as np

from revolve2.nca.core.actor.individual import Individual


class Actors(List):

    def __init__(self, agents: List[Individual] = None):
        if agents is None:
            super().__init__()
        else:
            super().__init__(agents)

    def append(self, agent: Individual):
        super().append(agent)

    def remove(self, agent: Individual):
        super().remove(agent)

    def __repr__(self):
        string_representation: string = "Agents {"

        for individual in self:
            string_representation += repr(individual) + ","

        string_representation += "}"

        return string_representation

    # TODO simplify and congregate with fitness_statistics call
    def find_typical_individuals(self):
        fitness_sorted_indexes = np.argsort([individual.fitness.value() for individual in self])
        number_of_quartiles = 5
        number_of_individuals = self.__len__()
        quartile_indexes = [round(percentile / (number_of_quartiles-1) * (number_of_individuals - 1)) for index, percentile in enumerate(range(number_of_quartiles))]
        labels = ['min', 'first', 'median', 'third', 'max']
        return {labels[index]: fitness_sorted_indexes[quartile_index]
                for index, quartile_index in enumerate(quartile_indexes)}

    def average_fitness(self):
        return np.mean([individual.fitness.value() for individual in self])

    def update_age(self):
        for agent in self:
            agent.age.update()
