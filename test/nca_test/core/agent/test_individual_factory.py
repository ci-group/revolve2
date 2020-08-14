import unittest

from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory


class TestIndividualFactory(unittest.TestCase):

    def test_compare(self):
        n = 5
        factory = IndividualFactory()

        individuals = factory.create(n)

        self.assertIsInstance(individuals, Agents)
        self.assertTrue(len(individuals), n)
        self.assertNotEqual(individuals[0].representation, individuals[1].representation)
        self.assertEqual(individuals[0].fitness, individuals[1].fitness)
