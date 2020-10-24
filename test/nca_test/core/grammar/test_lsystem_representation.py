import unittest

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.genome.grammar.grammar import ReplacementRules, RewritingGrammar
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from nca.core.genome.representations.symbolic_representation import LSystemAlgorithm
from nca_test.core.grammar.test_alphabet import TestColorSymbol


class LSystemRepresentationTest(unittest.TestCase):

    def test_same(self):
        rules: ReplacementRules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        algorithm = LSystemAlgorithm(RewritingGrammar(rules), GrammarInitialization(TestColorSymbol))
        encoding = algorithm()

        outcome = [TestColorSymbol.RED for _ in range(RepresentationConfiguration().genome_size)]
        self.assertEqual(encoding, outcome)

    def test_array(self):
        rules: ReplacementRules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        algorithm = LSystemAlgorithm(RewritingGrammar(rules), GrammarInitialization(TestColorSymbol))
        encoding = algorithm()
        outcome = [TestColorSymbol.RED for _ in range(len(encoding))]
        self.assertEqual(encoding, outcome)
