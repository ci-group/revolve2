from typing import List

from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.age import Age
from pyrevolve.evolutionary.fitness import Fitness, TestFitness
from pyrevolve.shared.configurations import EvolutionaryConfiguration
from pyrevolve.shared.sequential_identifier import AgentIdentifier


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, representation: Representation, fitness: Fitness):
        super().__init__()
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        self.representation = representation
        self.fitness: Fitness = fitness
