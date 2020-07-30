import unittest

from pyrevolve.evolutionary import Fitness, Agents
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.individual_factory import IndividualFactory


class TestIndividualFactory(unittest.TestCase):

    def test_compare(self):
        n = 5
        factory = IndividualFactory(Representation, Fitness)

        individuals = factory.create(n)

        self.assertIsInstance(individuals, Agents)
        self.assertTrue(len(individuals), n)

    def test_representation(self):
        n = 2
        factory = IndividualFactory(Representation, Fitness)

        individuals = factory.create(n)

        self.assertNotEqual(individuals[0].representation, individuals[1].representation)

    def test_fitness(self):
        n = 2
        factory = IndividualFactory(Representation, Fitness)

        individuals = factory.create(n)

        self.assertNotEqual(individuals[0].fitness, individuals[1].fitness)
