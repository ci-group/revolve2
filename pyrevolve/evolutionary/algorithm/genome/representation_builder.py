from abc import abstractmethod

from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representation import Representation


class RepresentationBuilder:

    def __init__(self, representation_type: type(Representation), mutation: MutationOperator, recombination: RecombinationOperator):
        self.representation_type = representation_type
        self.mutation: MutationOperator = mutation
        self.recombination: RecombinationOperator = recombination

    @abstractmethod
    def generate(self):
        return self.representation_type(self.mutation, self.recombination)
