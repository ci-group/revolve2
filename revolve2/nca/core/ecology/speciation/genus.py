from typing import List

from revolve2.abstract.sequential_identifier import GenusIdentifier
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.ecology.speciation.compatibility import Compatibility


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
                population.individuals.append(individual)
                return True

        return False

    def to_json(self):
        return {
            'id': self.id,
            'species': [
                species.to_json() for species in self.species
            ]
        }
