from typing import List

from nca.core.genome.genotype import Genotype
from nca.core.genome.grammar.grammar import RewritingGrammar
from nca.core.genome.grammar.grammar_initialization import LSystemInitialization
from nca.core.genome.grammar.symbol import Symbol
from nca.core.genome.representations.symbolic_representation import SymbolicRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenSymbol


class LSystemGenotype(Genotype):

    def __init__(self, initialization: LSystemInitialization = None):
        super().__init__(None)
        self.initialization = initialization
        self.initialize()

        self.grammar = RewritingGrammar()
        # get initialized symbols as the starting encoding
        self.encoding: List[Symbol] = self['axiom']

    def initialize(self):
        self.clear()
        representations = [self.initialization.axiom_initialization(RobogenSymbol, 0),
                           self.initialization.rules_initialization(RobogenSymbol, 0)]
        self._initialize_multiple_representations(representations, ['axiom', 'rules'])

    def __call__(self, rule_iterations: int = 1) -> List[Symbol]:
        for _ in range(rule_iterations):
            self.encoding = self.grammar.apply_rules(self['rules'], self.initialization.symbol_type)

        return self.encoding


class SelfOrganizingLSystemGenotype(Genotype):

    def __init__(self, initialization: LSystemInitialization = None):
        super().__init__(None)

        self.encoding: List[Symbol] = None

        self.initialization = initialization
        self.initialize()

        self.grammar = RewritingGrammar()

    def initialize(self):
        self.clear()
        symbols = SymbolicRepresentation(self.initialization.symbol_type)
        representations = [symbols,
                           self.initialization.axiom_initialization(symbols, 0),
                           self.initialization.rules_initialization(symbols, 0)]
        self._initialize_multiple_representations(representations, ['symbols', 'axiom', 'rules'])

        # get initialized symbols as the starting encoding
        self.encoding = self['axiom']

    def __call__(self, rule_iterations: int = 1) -> List[Symbol]:
        for _ in range(rule_iterations):
            self.encoding = self.grammar.apply_rules(self['rules'], self['symbols'])

        return self.encoding
