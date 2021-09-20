import unittest
from typing import List

from revolve2.nca.core.genome.grammar.grammar_initialization import GrammarInitialization
from test.nca.core.grammar.test_alphabet import TestColorSymbol


class TestGrammarInitialization(unittest.TestCase):

    def test_initialization_probability(self):
        initialization = GrammarInitialization(TestColorSymbol)

        genome: List = initialization(1000)

        counters: [TestColorSymbol, int] = {}
        for element in TestColorSymbol:
            counters[element] = 0

        for element in genome:
            counters[element] += 1
            self.assertIsInstance(element, TestColorSymbol)

        self.assertTrue(counters[TestColorSymbol.RED] > counters[TestColorSymbol.GREEN])
        self.assertTrue(counters[TestColorSymbol.RED] > counters[TestColorSymbol.BLUE])
        self.assertTrue(counters[TestColorSymbol.GREEN] > counters[TestColorSymbol.BLUE])
