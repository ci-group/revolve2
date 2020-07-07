from typing import List

from pyrevolve.agent.agent import Agent
from pyrevolve.agent.agents import Agents
from pyrevolve.population_ecology.population import Population


class Genus:

    def __init__(self, agents: Agents, id: int = 0):
        super().__init__(id, agents)
        self.id: int = id
        self.species: List[Population] = []

    def add(self, population: Population):
        self.species.append(population)

    def remove(self, population: Population):
        self.species.remove(population)

    def compatible(self, population: Population, candidate: Agent):
        #TODO Comparison
        pass

    def load(self):
        # TODO Load
        pass

    def export(self):
        # TODO Export
        pass