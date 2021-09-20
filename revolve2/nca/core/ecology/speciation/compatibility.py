from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual import Individual


class Compatibility:

    def __init__(self, limit: float = 5.0):
        self.limit = limit

    def compatible(self, individuals: Actors, other_individual: Individual) -> bool:
        sum_difference: float = 0.0
        for individual in individuals:
            sum_difference += individual.genotype.compatibility(other_individual.genotype)
        average_difference = sum_difference / len(individuals)
        return abs(average_difference) < self.limit
