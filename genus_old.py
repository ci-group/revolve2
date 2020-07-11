from typing import List

from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.ecology import Population


class Genus:

    def __init__(self, genus_id: int, agents: Agents = Agents()):
        super().__init__(id, agents)
        self.id: int = genus_id
        self.species: List[Population] = []

    def add(self, population: Population):
        self.species.append(population)

    def remove(self, population: Population):
        self.species.remove(population)

    def assign(self, agent):
        for population in self.species:
            if population.compatible(agent):
                return

