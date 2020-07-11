from typing import List

from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.shared.configurations import SpeciationConfiguration
from pyrevolve.ecology import Population
from pyrevolve.ecology import PopulationManagement
from pyrevolve.evolutionary.algorithm import Selection
from pyrevolve.ecology import Genus


class SpeciatedManagement(PopulationManagement):

    def __init__(self, selection: Selection, configuration=SpeciationConfiguration()):
        super().__init__(selection, configuration)

        self.genus: Genus = None

    def populations(self) -> List[Population]:
        if self.genus is None:
            raise Exception("Genus uninitialized")

        populations: List[Population] = []

        for population_management in self.genus.species:
            populations.append(population_management.population)

        return populations

    def create(self, agents: Agents):
        self.genus = Genus(self.identifier.increment())

        for agent in agents:
            self.assign(agent)

    def assign(self, agent):
        inserted = self.genus.insert(agent)

        if not inserted:
            population_management = PopulationManagement(self.selection)
            self.genus.add(population_management)

    def select(self):
        if self.genus is None:
            raise Exception("Genus is uninitialized")

        for specie in self.genus.species:
            specie.select()

    def speciate(self):
        agents: Agents = self.agents()

        self.create(agents)

