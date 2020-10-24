from typing import List

from nca.core.genome.grammar.grammar import RewritingGrammar, ReplacementRules, Symbol
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from nca.core.genome.representations.symbolic_representation import LSystemAlgorithm
from revolve.robot.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.robogen.robogen_module import RobogenModule

RobogenAxiom = List[RobogenModule]


class LSystemInitialization(GrammarInitialization):
    def __init__(self, symbol_type: type(Symbol), axiom=None):
        super().__init__(symbol_type)
        self.axiom = axiom

    def __call__(self, size: int = 0) -> List:
        if size == 1:  # TODO more generic
            return super().__call__(size)

        return self.axiom


class RobogenConstruction(LSystemAlgorithm):

    def __init__(self, rules: ReplacementRules = None, axiom=None):
        if rules is None:
            rules = RobogenSymbol.generate_unconstrained_rules()
        if axiom is None:
            axiom = RobogenSymbol.generate_axiom()
        super().__init__(grammar=RewritingGrammar(rules),
                         initialization=LSystemInitialization(RobogenSymbol, axiom))
