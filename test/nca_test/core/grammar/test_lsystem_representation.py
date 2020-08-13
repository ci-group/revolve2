import unittest

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.genome.grammar.grammar import ReplacementRules, Grammar
from nca.core.genome.representations.symbolic_representation import LSystemRepresentation
from nca_test.core.grammar.test_alphabet import TestColorSymbol


class LSystemRepresentationTest(unittest.TestCase):

    def test_same(self):
        rules: ReplacementRules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        representation = LSystemRepresentation(Grammar(TestColorSymbol, rules))
        representation.algorithm()

        outcome = [TestColorSymbol.RED for _ in range(RepresentationConfiguration().genome_size)]
        self.assertEqual(representation.genome, outcome)

    def test_array(self):
        rules: ReplacementRules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        representation = LSystemRepresentation(Grammar(TestColorSymbol, rules))
        representation.algorithm()

        outcome = [TestColorSymbol.RED for _ in range(len(representation.genome))]
        self.assertEqual(representation.genome, outcome)
