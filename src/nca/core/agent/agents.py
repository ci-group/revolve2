from typing import List
import numpy as np

from nca.core.abstract.iterator import Iterator
from nca.core.agent.individual import Individual


class Agents(Iterator):

    def __init__(self, agents: List[Individual] = None):
        super().__init__(agents)

    def get(self, index) -> Individual:
        return self.collection[index]

    def add(self, agent: Individual):
        super().add(agent)

    def remove(self, agent: Individual):
        super().remove(agent)

    def extend(self, agents):
        return self.collection.extend(agents.collection)

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

        for agent in self:
            if worst_fitness < agent.fitness:
                worst_fitness = agent.fitness
                worst_agent = agent

        return worst_agent
    """

    def average_fitness(self):
        # TODO individual.fitness
        return np.mean([individual.fitness.fitness if individual.fitness is not None else 0.0 for individual in self])

    def update_age(self):
        for agent in self:
            agent.age.update()
