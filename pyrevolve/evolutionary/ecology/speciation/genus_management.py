from typing import List

from pyrevolve.shared.configurations import SpeciationConfiguration
from pyrevolve.evolutionary.algorithm.selection.selection import Selection

from pyrevolve.evolutionary import Agents

from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from .genus import Genus
from .speciated_population import SpeciatedPopulation


class GenusManagement(PopulationManagement):

    def __init__(self, configuration=SpeciationConfiguration()):
        super().__init__(configuration)

        self.genus: Genus = None

    def population(self) -> List[Population]:
        if self.genus is None:
            raise Exception("Genus uninitialized")

        return [speciated_population for speciated_population in self.genus.species]
    """
    def agents(self) -> Agents:
        all_agents: Agents = Agents()

        for population in self.populations():
            for agent in population.individuals:
                all_agents.add(agent)

        return all_agents
    """
    def speciate(self):
        agents: Agents = self.agents()

        self.create(agents)

    def initialize(self, agents: Agents):
        self.genus = Genus()

        for agent in agents:
            self.assign(agent)

    def assign(self, agent):
        inserted = self.genus.insert(agent)

        if not inserted:
            speciated_population = SpeciatedPopulation(Agents([agent]))
            self.genus.add(speciated_population)
