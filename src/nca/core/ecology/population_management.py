from typing import List

from nca.core.abstract.configurations import PopulationConfiguration
from nca.core.agent.agents import Agents
from nca.core.ecology.population import Population


class PopulationManagement:

    def __init__(self, configuration: PopulationConfiguration = PopulationConfiguration()):
        super().__init__()
        self.configuration: PopulationConfiguration = configuration

        self.population: Population = None

    def populations(self) -> List[Population]:
        if self.population is not None:
            return [self.population]

        return None

    def initialize(self, agents: Agents):
        assert(len(agents) > 0)
        self.population = Population(agents)

    def speciate(self):
        pass
