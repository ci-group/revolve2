from typing import List

from nca.core.genome.representations.symbolic_representation import LSystemRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenModule, RobogenSymbol, RobogenGrammar, RobogenReplacementRules

RobogenAxiom = List[RobogenModule]


class RewritableRobogenRepresentation(LSystemRepresentation):

    def __init__(self, grammar: RobogenGrammar = None):
        if grammar is None:
            grammar = self.generate_grammar()

        axiom = self.generate_axiom()

        super().__init__(grammar, axiom)

    def generate_grammar(self) -> RobogenGrammar:
        replacement_rules: RobogenReplacementRules = dict()
        for symbol in RobogenSymbol.modules():

            replacement_rules[symbol] = [RobogenSymbol.random_words(3)]

        return RobogenGrammar(replacement_rules)

    def generate_axiom(self):
        axiom = [{'module': RobogenSymbol.MODULE_CORE}]

        for i in range(4):
            replacement_sequence: List[RobogenSymbol] = RobogenSymbol.random_words()
            replacement_sequence.insert(0, {'module': RobogenSymbol.BRACKET_STASH})
            replacement_sequence.append({'module': RobogenSymbol.BRACKET_POP})
            axiom.extend(replacement_sequence)

        return axiom
