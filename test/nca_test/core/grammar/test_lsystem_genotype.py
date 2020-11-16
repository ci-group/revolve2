import unittest

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.genome.grammar.grammar import RewritingGrammar
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from nca.core.genome.grammar.lindenmayer_system import LSystemGenotype
from nca_test.core.grammar.test_alphabet import TestColorSymbol

# TODO refactor
class LSystemGenotypeTest(unittest.TestCase):

    def test_same(self):
        rules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        algorithm = LSystemGenotype(RewritingGrammar(rules), GrammarInitialization(TestColorSymbol))
        encoding = algorithm()

        outcome = [TestColorSymbol.RED for _ in range(RepresentationConfiguration().genome_size)]
        self.assertEqual(encoding, outcome)

    def test_array(self):
        rules = {TestColorSymbol.GREEN: [[TestColorSymbol.RED]],
                                  TestColorSymbol.BLUE: [[TestColorSymbol.RED]]}

        algorithm = LSystemGenotype(RewritingGrammar(rules), GrammarInitialization(TestColorSymbol))
        encoding = algorithm()
        outcome = [TestColorSymbol.RED for _ in range(len(encoding))]
        self.assertEqual(encoding, outcome)
