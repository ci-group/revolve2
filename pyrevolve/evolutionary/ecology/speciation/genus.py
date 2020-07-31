from typing import List

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.speciation.compatibility import Compatibility
from pyrevolve.shared.sequential_identifier import GenusIdentifier


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
            if self.compatibility.compare(population.individuals, individual):
                population.individuals.add(individual)
                return True

        return False
