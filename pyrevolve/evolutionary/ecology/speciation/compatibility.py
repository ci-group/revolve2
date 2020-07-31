from pyrevolve.evolutionary import Agents, Individual


class Compatibility:

    def __init__(self, limit: float = 1.0):
        self.limit = limit

    def compare(self, individuals: Agents, other_individual: Individual) -> float:
        sum_difference: float = 0.0
        for individual in individuals:
            sum_difference += individual.representation.compatibility(other_individual.representation)
        average_difference = sum_difference / len(individuals)
        return average_difference
