from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.selection.selection import Selection
from pyrevolve.shared.configurations import PopulationConfiguration
from pyrevolve.shared.sequential_identifier import SequentialIdentifier
from pyrevolve.evolutionary.algorithm.ecology.population import Population


class PopulationManagement:

    def __init__(self, selection: Selection, configuration: PopulationConfiguration = PopulationConfiguration()):
        super().__init__()

        self.selection: Selection = selection
        self.configuration: PopulationConfiguration = configuration

        self.population: Population = None
        self.identifier: SequentialIdentifier = SequentialIdentifier()

    def populations(self):
        return [self.population]

    def create(self, agents: Agents):
        self.population = Population(self.identifier.increment(), agents)

    def select(self, algorithm_function):
        if self.population is None:
            raise Exception("Population is uninitialized")

        selected_agents = algorithm_function(self.population.parents)

        self.create(selected_agents)

    def speciate(self):
        raise Exception('Call speciation for non-speciation population management')

