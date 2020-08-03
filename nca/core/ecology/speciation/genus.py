from typing import List

from nca.core.agent import Individual
from nca.core.ecology.population import Population
from nca.experiment.sequential_identifier import GenusIdentifier
from nca.core.ecology.speciation.compatibility import Compatibility


class Genus:

    identifier = GenusIdentifier()

    def __init__(self, compatibility: Compatibility = Compatibility()):
        self.id: int = self.identifier.id()
        self.compatibility = compatibility

        self.species: List[Population] = []

    def add(self, population: Population):
        self.species.append(population)

    def insert(self, individual: Individual):
        for population in self.species:
            if self.compatibility.compatible(population.individuals, individual):
                population.individuals.add(individual)
                return True

        return False
