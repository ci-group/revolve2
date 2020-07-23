from pyrevolve.evolutionary.ecology.speciation.compatibility import Compatibility
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary import Agents, Individual


class SpeciatedPopulation(Population):

    compatibility: Compatibility = Compatibility()

    def __init__(self, individuals: Agents):
        super().__init__(individuals)

        self.representative = None

    def compatible(self, individual: Individual):
        if self.compatibility(self.individuals, individual):
            self.individuals.add(individual)
            return True

        return False
