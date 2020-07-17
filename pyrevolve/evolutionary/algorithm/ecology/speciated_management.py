from typing import List

from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.shared.configurations import SpeciationConfiguration
from pyrevolve.evolutionary.algorithm.ecology.population import Population
from pyrevolve.evolutionary.algorithm.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.algorithm import Selection
from pyrevolve.evolutionary.algorithm.ecology import Genus


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

    def select(self, algorithm_function):
        if self.genus is None:
            raise Exception("Genus is uninitialized")

        for species in self.genus.species:
            species.select(algorithm_function)


    def agents(self) -> Agents:
        all_agents: Agents = Agents()

        for population in self.populations():
            for agent in population.parents:
                all_agents.add(agent)

        return all_agents

    def speciate(self):
        agents: Agents = self.agents()

        self.create(agents)

    def create(self, agents: Agents):
        self.genus = Genus(self.identifier.increment())

        for agent in agents:
            self.assign(agent)

    def assign(self, agent):
        inserted = self.genus.insert(agent)

        if not inserted:
            population_management = PopulationManagement(self.selection)
            self.genus.add(population_management)
