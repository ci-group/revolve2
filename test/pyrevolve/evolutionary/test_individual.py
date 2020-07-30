import unittest
from typing import List

from pyrevolve.evolutionary import Fitness
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.individual import Individual


class TestIndividual(unittest.TestCase):

    def test_compare(self):

        representation = Representation()
        fitness = Fitness()
        individual = Individual(representation, fitness)

        self.assertEqual(individual.representation, representation)
        self.assertEqual(individual.fitness, fitness)
