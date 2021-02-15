import unittest

from nca.core.actor.actors import Actors
from nca.core.actor.individual_factory import ActorFactory


class TestIndividualFactory(unittest.TestCase):

    def test_individual(self):
        n = 2
        factory = ActorFactory()

        individuals = factory.create(n)

        self.assertIsInstance(individuals, Actors)
        self.assertTrue(len(individuals), n)
        self.assertNotEqual(individuals[0].get_representation(), individuals[1].get_representation())
        self.assertEqual(individuals[0].fitness.value(), individuals[1].fitness.value())
