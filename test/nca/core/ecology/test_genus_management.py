import unittest

import numpy as np

from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology.speciation.compatibility import Compatibility
from revolve2.nca.core.ecology.speciation.genus_management import GenusManagement


class GenusManagementTest(unittest.TestCase):

    def test_management(self):
        population_management = GenusManagement()

        agents1: Actors = ActorFactory().create(3)
        for agent in agents1:
            agent.get_representation()[:] = [0, 0, 0, 0]
        population_management.initialize(agents1)

        for index in range(len(agents1)):
            self.assertEqual(population_management.genus.species[0].individuals[index], agents1[index])

        agents2: Actors = ActorFactory().create(3)
        for agent in agents2:
            agent.get_representation()[:] = [0, 0, 0, 0]
            population_management.assign(agent)

        agents1.extend(agents2)
        for index in range(len(agents1)):
            self.assertEqual(population_management.genus.species[0].individuals[index], agents1[index])

    def test_basics(self):
        population_management = GenusManagement(compatibility=Compatibility(1))

        factory = ActorFactory()
        agents: Actors = factory.create(3)

        agents[0].get_representation()[:] = [0, 0, 0, 0]
        agents[1].get_representation()[:] = [0, 0, 0, 0]
        agents[2].get_representation()[:] = [0, 2, 0, 0]
        population_management.initialize(agents)

        self.assertEqual(len(population_management.genus.species), 2)

    def test_speciation(self):
        population_management = GenusManagement(compatibility=Compatibility(1))

        factory = ActorFactory()
        agents: Actors = factory.create(3)
        agents[0].get_representation()[:] = [0, 0, 0, 0]
        agents[1].get_representation()[:] = [0, 0, 0, 0]
        agents[2].get_representation()[:] = [0, 0, 0, 0]
        population_management.initialize(agents)

        self.assertEqual(len(population_management.genus.species), 1)

        population_management.genus.species[0].individuals[0].get_representation()[:] = [0, 2, 0, 0]
        population_management.speciate()

        self.assertEqual(len(population_management.genus.species), 2)
