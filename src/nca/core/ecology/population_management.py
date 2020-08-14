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
        if self.population is None:
            raise Exception("PopulationManagement: Not existing population")
        return [self.population]

    def initialize(self, agents: Agents):
        assert(len(agents) > 0)
        self.population = Population(agents)

    def speciate(self):
        pass

    """
    def filter(self, condition: Condition):
        filtered_individuals: Agents = condition.filter(self.population)

        for individual in filtered_individuals:
            self.population.remove(individual)

        return filtered_individuals

    def transfer(self, agents: Agents):
        for individual in agents:
            self.population.add(individual)
    """