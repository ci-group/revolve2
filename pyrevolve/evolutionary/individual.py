from typing import List

from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.age import Age
from pyrevolve.evolutionary.fitness import Fitness, TestFitness
from pyrevolve.shared.configurations import EvolutionaryConfiguration
from pyrevolve.shared.sequential_identifier import AgentIdentifier


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, representation: Representation = None, parents: List[object] = [], fitness: Fitness = EvolutionaryConfiguration().fitness):
        super().__init__()
        self.id: int = self.identifier.id()
        self.age: Age = Age()
        # self.name = random_name_generator.get(agent_id)

        self.representation = representation
        self.fitness: Fitness = fitness

        self.parents: List[Individual] = parents


class TestIndividual(Individual):
    def __init__(self):
        super().__init__(fitness=TestFitness())
