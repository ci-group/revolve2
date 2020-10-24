import unittest

from nca.core.actor.actors import Actors
from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology.population_management import PopulationManagement


class PopulationManagementTest(unittest.TestCase):
    n=3

    def test_management(self):
        population_management = PopulationManagement()

        agents1: Actors = ActorFactory().create(self.n)
        population_management.initialize(agents1)

        self.assertEqual(population_management.population.individuals, agents1)

        agents2: Actors = ActorFactory().create(self.n)
        population_management.initialize(agents2)

        self.assertEqual(population_management.population.individuals, agents2)
