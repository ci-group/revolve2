import string

from .population_management import PopulationManagement
from nca.core.abstract.behavioral.memento import Memento
from ..actor.actors import Actors


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement = PopulationManagement()):
        super().__init__()
        self.management: PopulationManagement = population_management

    def initialize(self, agents: Actors):
        self.management.initialize(agents)

    def speciate(self):
        self.management.speciate()

    def export(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.dump(path, self.management)

    def load(self, path: string = ""):
        path = self.population_memento_path if path == "" else path
        self.management = self.restore(path)
