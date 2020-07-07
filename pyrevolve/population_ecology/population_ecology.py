import pickle

from pyrevolve.agent.agents import Agents
from pyrevolve.patterns.abstract.memento import Memento
from pyrevolve.population_ecology.population import Population
from pyrevolve.population_ecology.population_management import PopulationManagement


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement):
        super().__init__()
        self.management: PopulationManagement = population_management

    def create(self, agents: Agents) -> Population:
        return self.management.create(agents)

    def select(self) -> Population:
        return self.management.select()

    def speciate(self) -> Population:
        return self.management.speciate()

    def save(self):
        self.management = pickle.dump(self.management, file = open(self.population_memento_path, "rb"))

    def recover(self):
        self.management = pickle.load(open(self.population_memento_path, "wb"))
        return self.management