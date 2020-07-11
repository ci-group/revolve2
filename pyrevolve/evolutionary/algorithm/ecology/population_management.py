from pyrevolve.evolutionary.robotics import Agents
from pyrevolve.shared.configurations import PopulationConfiguration
from pyrevolve.shared.sequential_identifier import SequentialIdentifier
from pyrevolve.evolutionary.algorithm.ecology import Population
from pyrevolve.evolutionary.algorithm import Selection


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
        self.population = Population(self.identifier.increment(), agents, self.configuration.path)

    def select(self, algorithm_function):
        if self.population is None:
            raise Exception("Population is uninitialized")

        selected_agents = algorithm_function(self.population.agents)

        self.create(selected_agents)

    def agents(self) -> Agents:
        all_agents: Agents = Agents()

        for population in self.populations():
            for agent in population:
                all_agents.add(agent)

        return all_agents

    def speciate(self):
        raise Exception('Call speciation for non-speciation population management')

