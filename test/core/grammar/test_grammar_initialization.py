import unittest

from nca.core.genome.representation import Genome
from nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from test.core.grammar.test_alphabet import TestColorAlphabet


class TestGrammarInitialization(unittest.TestCase):

    def test_initialization_probability(self):
        initialization = GrammarInitialization(TestColorAlphabet)

        genome: Genome = initialization.algorithm(1000)

        counters: [TestColorAlphabet, int] = {}
        for element in TestColorAlphabet.list():
            counters[element] = 0

        for element in genome:
            counters[element] += 1
            self.assertIsInstance(element, TestColorAlphabet)

        self.assertTrue(counters[TestColorAlphabet.RED] > counters[TestColorAlphabet.GREEN])
        self.assertTrue(counters[TestColorAlphabet.RED] > counters[TestColorAlphabet.BLUE])
        self.assertTrue(counters[TestColorAlphabet.GREEN] > counters[TestColorAlphabet.BLUE])
