import unittest

from nca.core.actor.actors import Actors
from nca.core.actor.individual_factory import IndividualFactory


class TestIndividualFactory(unittest.TestCase):

    def test_individual(self):
        n = 5
        factory = IndividualFactory()

        individuals = factory.create(n)

        self.assertIsInstance(individuals, Actors)
        self.assertTrue(len(individuals), n)
        self.assertNotEqual(individuals[0].representation, individuals[1].representation)
        self.assertEqual(individuals[0].fitness, individuals[1].fitness)
