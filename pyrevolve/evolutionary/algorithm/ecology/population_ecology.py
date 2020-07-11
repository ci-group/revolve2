import os
import pickle
from typing import List

from pyrevolve.evolutionary.agents import Agents
from pyrevolve.shared.abstract.memento import Memento
from pyrevolve.evolutionary.algorithm.ecology import Population
from pyrevolve.evolutionary.algorithm.ecology import PopulationManagement


class PopulationEcology(Memento):

    def __init__(self, population_management: PopulationManagement):
        super().__init__()
        self.management: PopulationManagement = population_management

    def create(self, agents: Agents):
        self.management.create(agents)

    def select(self, algorithm_function):
        self.management.select(algorithm_function)

    def speciate(self):
        self.management.speciate()

    def populations(self) -> List[Population]:
        return self.management.populations()

    def export(self):
        with open(self.population_memento_path, "wb") as f:
            pickle.dump(self.management, f, pickle.HIGHEST_PROTOCOL)

    def load(self):
        if os.path.exists(self.population_memento_path):
            with open(self.population_memento_path, "rb") as f:
                self.management = pickle.load(f)
