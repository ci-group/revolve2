from pyrevolve.agent.agents import Agents
from pyrevolve.patterns.abstract.memento import Memento
from pyrevolve.patterns.configuration import PopulationConfiguration
from pyrevolve.patterns.sequential_identifier import PopulationIdentifier
from pyrevolve.population_ecology.population import Population
from pyrevolve.population_ecology.selection import Selection
from pyrevolve.experiment.experiment_manager import ExperimentManager


class PopulationManagement:

    def __init__(self, configuration: PopulationConfiguration, selection: Selection):
        super().__init__()
        self.configuration: PopulationConfiguration = configuration
        self.identifier: PopulationIdentifier = PopulationIdentifier()
        self.population: Population = None
        self.selection: Selection = selection

    def create(self, agents: Agents) -> Population:
        self.population = Population(self.identifier.increment(), agents, self.configuration.path)
        return self.population

    def select(self) -> Population:
        selected_agents = self.selection.select(self.population.agents)
        self.population = Population(self.identifier.increment(), selected_agents, self.configuration.path)
        return self.population

    def speciate(self):
        raise Exception('Call speciation for non-speciation population management')
