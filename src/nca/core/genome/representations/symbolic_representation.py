from typing import List

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.grammar import RewritingGrammar, Symbol
from nca.core.genome.representation import Representation


# Or grammar Representation
class SymbolicRepresentation(Representation):

    def __init__(self, initialization: Initialization = None):
        super().__init__(initialization)

    def compatibility(self, other) -> float:
        differences = 0

        for index, element in enumerate(self):
            if self[index] != other[index]:
                differences += 1

        return differences

    def visit(self, representation_visitor):
        representation_visitor.visit_symbolic_representation(self)

    def random_value(self):
        return self.initialization(1)


class LSystemAlgorithm:

    def __init__(self, grammar: RewritingGrammar, initialization: Initialization = None):
        self.representation = SymbolicRepresentation(initialization)
        self.grammar: RewritingGrammar = grammar
        self.encoding: List[Symbol] = []

    def __call__(self, rule_iterations: int = 1):
        self.encoding = self.representation
        for _ in range(rule_iterations):
            self.encoding = self.grammar.apply_rules(self.encoding)
        return self.encoding