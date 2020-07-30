from typing import List

from pyrevolve.evolutionary.agents import Agents
from pyrevolve.shared.configurations import PopulationConfiguration
from pyrevolve.evolutionary.ecology.population import Population


class PopulationManagement:

    def __init__(self, configuration: PopulationConfiguration = PopulationConfiguration()):
        super().__init__()
        self.configuration: PopulationConfiguration = configuration

        self.population: Population = None

    def populations(self) -> List[Population]:
        return [self.population]

    def create_population(self, agents: Agents):
        self.population = Population(agents)


class TestPopulationManagement(PopulationManagement):

    def __init__(self):
        super().__init__(PopulationConfiguration())
