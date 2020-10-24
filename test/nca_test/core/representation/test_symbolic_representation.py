import unittest

from nca.core.genome.grammar.grammar import RewritingGrammar, ReplacementRules
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from nca.core.genome.representations.symbolic_representation import LSystemAlgorithm
from nca_test.core.grammar.test_alphabet import TestColorSymbol


class LSystemTest(unittest.TestCase):

    def test_lsystem(self):
        representation_1 = LSystemAlgorithm(RewritingGrammar(ReplacementRules), GrammarInitialization(TestColorSymbol))
        representation_2 = LSystemAlgorithm(RewritingGrammar(ReplacementRules), GrammarInitialization(TestColorSymbol))

        self.assertNotEqual(representation_1, representation_2)
