import unittest
from typing import List

import numpy as np

from nca.core.genome.initialization import CategoricalInitialization, UniformInitialization, \
    GaussianInitialization
from nca.core.genome.representations.l_system.alphabet import TestColorAlphabet


class TestInitializations(unittest.TestCase):

    def test_categorical(self):
        elements = TestColorAlphabet.list()
        initialization = CategoricalInitialization(len(elements))

        n = 10
        result = [TestColorAlphabet(element) for element in initialization.algorithm(n)]

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
