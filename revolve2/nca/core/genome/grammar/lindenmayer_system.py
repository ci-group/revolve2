from typing import List

from revolve2.nca.core.genome.genotype import Genotype
from revolve2.nca.core.genome.grammar.grammar import RewritingGrammar
from revolve2.nca.core.genome.grammar.grammar_initialization import LSystemInitialization, GrammarInitialization
from revolve2.nca.core.genome.grammar.symbol import Symbol
from revolve2.nca.core.genome.representations.symbolic_representation import SymbolicRepresentation
from revolve2.revolve.robot.body.robogen.robogen_grammar import RobogenSymbol


class LSystemGenotype(Genotype):

    def __init__(self, initialization: LSystemInitialization = None, grammar: RewritingGrammar = RewritingGrammar()):
        super().__init__(None)
        self.initialization = initialization
        self.initialize()

        self.grammar = grammar
        # get initialized symbols as the starting encoding
        self.encoding: List[Symbol] = self['axiom']

    def initialize(self):
        self.clear()
        representations = [self.initialization.axiom_initialization(RobogenSymbol, 0) if self.initialization.axiom_initialization is not None else GrammarInitialization(self.initialization.symbol_type)(0),
                           self.initialization.rules_initialization(RobogenSymbol, 0) if self.initialization.rules_initialization is not None else None,]

        self._initialize_multiple_representations(representations, ['axiom', 'rules'])

    def __call__(self, rule_iterations: int = 1, rules=None) -> List[Symbol]:
        rules = self['rules'] if rules is None else rules
        for _ in range(rule_iterations):
            self.encoding = self.grammar.apply_rules(rules, self.initialization.symbol_type)

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
