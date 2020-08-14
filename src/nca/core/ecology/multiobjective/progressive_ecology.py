import string
from typing import List

from nca.core.ecology import PopulationEcology
from nca.core.ecology.population import Population
from nca.core.ecology.population_management import PopulationManagement
from nca.core.evolution.conditions.condition import Condition


class ProgressiveEcology(PopulationEcology):

    def __init__(self, population_managements: List[PopulationManagement], conditions: List[Condition]):
        super().__init__()
        self.managements: List[PopulationManagement] = population_managements
        self.conditions: List[Condition] = conditions

    def initialize(self, environment):
        for management in self.managements:
            management.initialize(environment)

    def populations(self):
        populations: List[Population] = []
        for management in self.managements:
            populations.extend(management.populations())
        return populations

    """
    def transfer(self):
        for index, management in enumerate(self.managements):
            agents = management.filter(self.conditions[index])
            self.managements[index].transfer(agents)
    """

    def speciate(self):
        for management in self.managements:
            management.speciate()

    def export(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.dump(path, self.managements)

    def load(self, path: string = ""):
        path = self.population_memento_path if path == "" else path
        self.managements = self.restore(path)
