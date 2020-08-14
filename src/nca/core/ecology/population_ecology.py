import string

from .population_management import PopulationManagement
from ..abstract.memento import Memento
from ..agent.agents import Agents


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement = PopulationManagement()):
        super().__init__()
        self.management: PopulationManagement = population_management

    def initialize(self, agents: Agents):
        self.management.initialize(agents)

    def speciate(self):
        self.management.speciate()

    def export(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.dump(path, self.management)

    def load(self, path: string = ""):
        path = self.population_memento_path if path == "" else path
        self.management = self.restore(path)
