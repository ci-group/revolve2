import string
from typing import List

from nca.core.abstract.memento import Memento

from nca.core.agent.agents import Agents
from .population import Population
from .population_management import PopulationManagement


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement):
        super().__init__()
        self.management: PopulationManagement = population_management

    def initialize(self, agents: Agents):
        self.management.initialize(agents)

    def speciate(self):
        self.management.speciate()

    def populations(self) -> List[Population]:
        return self.management.populations()

    def export(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.dump(path, self.management)

    def load(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.management = self.restore(path)
