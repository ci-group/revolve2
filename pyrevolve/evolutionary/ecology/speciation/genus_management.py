from typing import List

from pyrevolve.shared.configurations import SpeciationConfiguration

from pyrevolve.evolutionary import Agents, Individual

from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from .compatibility import Compatibility
from .genus import Genus


class GenusManagement(PopulationManagement):

    def __init__(self, configuration=SpeciationConfiguration(), compatibility: Compatibility = Compatibility()):
        super().__init__(configuration)

        self.genus: Genus = Genus(compatibility)

    def initialize(self, agents: Agents):
        self.genus = Genus(self.genus.compatibility)

        for agent in agents:
            self.assign(agent)

    def assign(self, individual: Individual):
        inserted = self.genus.insert(individual)

        if not inserted:
            self.genus.add(Population(Agents([individual])))

    def speciate(self):
        self.initialize(self.agents())

    def population(self) -> List[Population]:
        if self.genus is None:
            raise Exception("Genus uninitialized")

        return [speciated_population for speciated_population in self.genus.species]

    def agents(self) -> Agents:
        all_agents: Agents = Agents()

        for population in self.populations():
            for agent in population.individuals:
                all_agents.add(agent)

        return all_agents
