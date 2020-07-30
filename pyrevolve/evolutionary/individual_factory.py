from typing import List

from pyrevolve.evolutionary import Individual, Agents
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.age import Age
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import BinaryRepresentation
from pyrevolve.evolutionary.fitness import Fitness, TestFitness, DisplacementFitness
from pyrevolve.shared.configurations import EvolutionaryConfiguration
from pyrevolve.shared.sequential_identifier import AgentIdentifier


class IndividualFactory:

    def __init__(self, representation_type: type(Representation) = Representation, fitness_type: type(Fitness) = Fitness):
        super().__init__()

        self.representation_type: type(Representation) = representation_type
        self.fitness_type: type(Fitness) = fitness_type

    def create(self, n: int = 0) -> Agents:
        return Agents([Individual(self.representation_type(), self.fitness_type()) for _ in range(n)])


if __name__ == "__main__":
    factory = IndividualFactory(BinaryRepresentation, DisplacementFitness)

    individuals = factory.create(3)

    print(individuals)
