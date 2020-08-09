import random

from nca.core.genome.grammar.grammar import Grammar
from nca.core.genome.representation import Representation


class LSystemRepresentation(Representation):

    def __init__(self, grammar: Grammar):
        super().__init__()
        self.grammar = grammar
        self.init()

    def _initialize(self):
        self.genome = random.choices(self.grammar.alphabet, k=self.configuration.genome_size)

    def algorithm(self, rule_iterations: int = 1):
        for _ in range(rule_iterations):
            self.genome = self.grammar.apply_rules(self.genome)

    def compatibility(self, other) -> float:
        differences = 0

        for index, element in enumerate(self.genome):
            if self.genome[index] != other.genome[index]:
                differences += 1

        return differences

    def visit(self, representation_visitor):
        representation_visitor.visit_lsystem_representation(self)
