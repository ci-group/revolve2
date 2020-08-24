import numpy as np

from nca.core.genome.grammar.grammar import Symbol, Grammar
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from nca.core.genome.representation import Representation


class SymbolicRepresentation(Representation):

    def __init__(self, symbols_type: type(Symbol)):
        self.symbols_type = symbols_type
        super().__init__(GrammarInitialization(self.symbols_type))

    def compatibility(self, other) -> float:
        differences = 0

        for index, element in enumerate(self.genome):
            if self.genome[index] != other.genome[index]:
                differences += 1

        return differences

    def visit(self, representation_visitor):
        representation_visitor.visit_symbolic_representation(self)

    def random_value(self):
        return np.random.choice(self.symbols_type)


class LSystemRepresentation(SymbolicRepresentation):

    def __init__(self, grammar: Grammar):
        super().__init__(grammar.symbols_type)
        self.grammar = grammar

    def __call__(self, rule_iterations: int = 1):
        for _ in range(rule_iterations):
            self.genome = self.grammar.apply_rules(self.genome)
