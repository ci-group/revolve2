import unittest

from abstract.configurations import RepresentationConfiguration
from nca.core.genome.grammar.grammar import RewritingGrammar
from nca.core.genome.grammar.grammar_initialization import LSystemInitialization
from nca.core.genome.grammar.lindenmayer_system import LSystemGenotype
from test.nca.core.grammar.test_alphabet import TestColorSymbol


# TODO refactor
class LSystemGenotypeTest(unittest.TestCase):

    def test_same(self):
        rules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        algorithm = LSystemGenotype(LSystemInitialization(TestColorSymbol), RewritingGrammar())
        print(algorithm.encoding)
        encoding = algorithm(1, rules)

        print(algorithm.encoding)
        outcome = [TestColorSymbol.RED for _ in range(RepresentationConfiguration().genome_size)]
        print(outcome)
        self.assertEqual(encoding, outcome)

    def test_array(self):
        rules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        algorithm = LSystemGenotype(LSystemInitialization(TestColorSymbol), RewritingGrammar())
        encoding = algorithm(1, rules)
        outcome = [TestColorSymbol.RED for _ in range(len(encoding))]
        self.assertEqual(encoding, outcome)
