import unittest
from typing import List

from nca.core.genome.operators.initialization import CategoricalInitialization, UniformInitialization, \
    GaussianInitialization
from test.nca.core.grammar.test_alphabet import TestColorSymbol


class TestInitializations(unittest.TestCase):

    def test_categorical(self):
        elements = TestColorSymbol
        initialization = CategoricalInitialization(len(elements))

        n = 10
        result = [TestColorSymbol(element) for element in initialization(n)]

        self.assertIsInstance(result, List)
        self.assertEqual(len(result), n)
        for index in range(n):
            self.assertTrue(result[index] in elements)

    def test_uniform(self):
        initialization = UniformInitialization()

        n = 10
        result = initialization(n)

        self.assertIsInstance(result, List)
        self.assertEqual(len(result), n)

    def test_gaussian(self):
        initialization = GaussianInitialization()

        n = 10
        result = initialization(n)

        self.assertIsInstance(result, List)
        self.assertEqual(len(result), n)
