import random

from nca.core.genome.grammar.grammar import Symbol, Grammar
from nca.core.genome.representation import Representation


class SymbolicRepresentation(Representation):

    def __init__(self, symbols_type: type(Symbol)):
        self.symbols_type = symbols_type
        super().__init__()

    def _initialize(self):
        self.genome = random.choices(self.symbols_type.alphabet(), weights=self.symbols_type.probabilities(),
                                     k=self.configuration.genome_size)

    def compatibility(self, other) -> float:
        differences = 0

        for index, element in enumerate(self.genome):
            if self.genome[index] != other.genome[index]:
                differences += 1

        return differences

    def visit(self, representation_visitor):
        representation_visitor.visit_symbolic_representation(self)


class LSystemRepresentation(SymbolicRepresentation):

    def __init__(self, grammar: Grammar):
        super().__init__(grammar.symbols_type)
        self.grammar = grammar

    def algorithm(self, rule_iterations: int = 1):
        for _ in range(rule_iterations):
            self.genome = self.grammar.apply_rules(self.genome)

