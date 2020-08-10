import unittest

from nca.core.agent.agents import Agents
from nca.core.agent.fitness import Fitness
from nca.core.agent.individual_factory import IndividualFactory
from nca.core.genome.representation import Representation


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
