import string
import time

from .population_management import PopulationManagement
from nca.core.abstract.behavioral.memento import Memento
from ..actor.actors import Actors


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement = PopulationManagement()):
        super().__init__("population_management")
        self.management: PopulationManagement = population_management

        self.speciation_duration = []
        self.logging_duration = []
        self.exporting_duration = []

    def initialize(self, agents: Actors):
        self.management.initialize(agents)

    def process(self):
        start_time = time.time()
        self.management.speciate()
        speciation_time = time.time() - start_time
        self.speciation_duration.append(speciation_time)

        for population in self.management.populations():
            population.log()
        logging_time = time.time() - start_time
        self.logging_duration.append(logging_time)

    def export(self, path: string = ""):
        start_time = time.time()
        path = self.population_memento_path if path == "" else path
        self.dump(path, self.management)
        logging_time = time.time() - start_time
        self.logging_duration.append(logging_time)

    def load(self, path: string = ""):
        path = self.population_memento_path if path == "" else path
        self.management = self.restore(path)

    def to_json(self):
        return self.management.to_json()
