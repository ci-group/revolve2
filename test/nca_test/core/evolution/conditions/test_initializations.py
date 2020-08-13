import unittest
from typing import List

import numpy as np

from src.nca.core.genome.initialization import CategoricalInitialization, UniformInitialization, \
    GaussianInitialization
from nca_test.core.grammar.test_alphabet import TestColorSymbol


class TestInitializations(unittest.TestCase):

    def test_categorical(self):
        elements = TestColorSymbol.alphabet()
        initialization = CategoricalInitialization(len(elements))

        n = 10
        result = [TestColorSymbol(element) for element in initialization.algorithm(n)]

        self.assertIsInstance(result, List)
        self.assertEqual(len(result), n)
        for index in range(n):
            self.assertTrue(result[index] in elements)

    def test_uniform(self):
        initialization = UniformInitialization()

        n = 10
        result = initialization.algorithm(n)

        self.assertIsInstance(result, np.ndarray)
        self.assertEqual(len(result), n)

    def test_gaussian(self):
        initialization = GaussianInitialization()

        n = 10
        result = initialization.algorithm(n)

        self.assertIsInstance(result, np.ndarray)
        self.assertEqual(len(result), n)
