import os
import pickle
import string
from typing import List

from pyrevolve.shared.abstract.memento import Memento

from .. import Agents
from .population import Population
from .population_management import PopulationManagement


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement):
        super().__init__()
        self.management: PopulationManagement = population_management

    def initialize(self, agents: Agents):
        self.management.create_population(agents)

    def select(self, parent_algorithm):
        populations: List[Population] = self.management.populations()
        for population in populations:
            population.offspring = parent_algorithm(population.individuals)

    def survive(self, survival_algorithm):
        populations: List[Population] = self.management.populations()
        for population in populations:
            new_population = survival_algorithm(population.individuals, population.offspring)
            population.next_generation(new_population)

    def export(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.dump(path, self.management)

    def load(self, path: string = ""):
        path = self.population_memento_path if path == "" else path

        self.management = self.restore(path)
