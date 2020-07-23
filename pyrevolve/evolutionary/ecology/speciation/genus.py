from typing import List

from pyrevolve.shared.sequential_identifier import GenusIdentifier

from .speciated_population import SpeciatedPopulation


class Genus:

    identifier = GenusIdentifier()

    def __init__(self):
        self.id: int = self.identifier.id()
        self.species: List[SpeciatedPopulation] = []

    def add(self, speciated_population: SpeciatedPopulation):
        self.species.append(speciated_population)

    def remove(self, speciated_population: SpeciatedPopulation):
        self.species.remove(speciated_population)

    def insert(self, agent):
        for speciated_population in self.species:
            if speciated_population.compatible(agent):
                return True

        return False
