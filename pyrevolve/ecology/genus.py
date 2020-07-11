from typing import List

from pyrevolve.ecology import PopulationManagement


class Genus:

    def __init__(self, genus_id: int):
        self.id: int = genus_id
        self.species: List[PopulationManagement] = []

    def add(self, population_management: PopulationManagement):
        self.species.append(population_management)

    def remove(self, population: PopulationManagement):
        self.species.remove(population)

    def insert(self, agent):
        for population_management in self.species:
            if population_management.population.compatible(agent):
                return True

        return False
