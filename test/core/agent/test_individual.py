import unittest

from nca.core.agent import Fitness
from nca.core.genome.representation import Representation
from nca.core.agent import Individual


class TestIndividual(unittest.TestCase):

    def test_compare(self):

        representation = Representation()
        fitness = Fitness()
        individual = Individual(representation, fitness)

        self.assertEqual(individual.representation, representation)
        self.assertEqual(individual.fitness, fitness)
