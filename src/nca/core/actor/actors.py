import string
from typing import List
import numpy as np

from nca.core.abstract.behavioral.iterator import Iterator
from nca.core.actor.individual import Individual


class Actors(Iterator):

    def __init__(self, agents: List[Individual] = None):
        super().__init__(agents)

    def _get(self, index) -> Individual:
        return self.collection[index]

    def add(self, agent: Individual):
        super().add(agent)

    def remove(self, agent: Individual):
        super().remove(agent)

    def extend(self, agents):
        return self.collection.extend(agents.collection)

    def __repr__(self):
        string_representation: string = "Agents {"

        for individual in self.collection:
            string_representation += repr(individual) + ","

        string_representation += "}"

        return string_representation

    """
    def get_best(self) -> Individual:
        best_agent: Individual = None
        best_fitness: Fitness = Fitness.worst()

        for evolutionary_agent in self:
            if best_fitness > evolutionary_agent.fitness:
                best_fitness = evolutionary_agent.fitness
                best_agent = evolutionary_agent

        return best_agent

    def get_worst(self) -> Individual:
        worst_agent: Individual = None
        worst_fitness: Fitness = Fitness.best()

        for actor in self:
            if worst_fitness < actor.fitness:
                worst_fitness = actor.fitness
                worst_agent = actor

        return worst_agent
    """

    def average_fitness(self):
        return np.mean([individual.fitness for individual in self])

    def update_age(self):
        for agent in self:
            agent.age.update()
