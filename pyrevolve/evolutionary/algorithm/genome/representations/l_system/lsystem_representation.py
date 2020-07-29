import random
from abc import abstractmethod

from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.alphabet import Alphabet
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import GrammarRepresentation
from pyrevolve.evolutionary.algorithm.genome.representations.l_system.lsystem_rules import Rules


class LSystemRepresentation(GrammarRepresentation):

    def __init__(self, alphabet: type(Alphabet), rules: Rules):
        super().__init__(alphabet=alphabet)
        self.rules: Rules = rules

    def apply_rules(self):
        for index, element in enumerate(self.genome):
            if element in self.rules.keys():
                self.genome.remove(element)
                self.genome.insert(index, self.rules[element])

    def compatibility(self, other):
        differences = 0

        for index, element in enumerate(self.genome):
            if self.genome[index] != other.genome[index]:
                differences += 1

        return differences
