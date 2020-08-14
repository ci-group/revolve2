import unittest

import numpy as np

from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory
from nca.core.ecology.speciation.compatibility import Compatibility
from nca.core.ecology.speciation.genus_management import GenusManagement
from nca.core.genome.representations.valued_representation import BinaryRepresentation


class GenusManagementTest(unittest.TestCase):

    def test_management(self):
        population_management = GenusManagement()

        agents1: Agents = IndividualFactory(BinaryRepresentation).create(3)
        for agent in agents1:
            agent.representation.genome = np.array([0, 0, 0, 0])
        population_management.initialize(agents1)

        for index in range(len(agents1)):
            self.assertEqual(population_management.genus.species[0].individuals[index], agents1[index])

        agents2: Agents = IndividualFactory(BinaryRepresentation).create(3)
        for agent in agents2:
            agent.representation.genome = np.array([0, 0, 0, 0])
            population_management.assign(agent)

        agents1.extend(agents2)
        for index in range(len(agents1)):
            self.assertEqual(population_management.genus.species[0].individuals[index], agents1[index])

    def test_basics(self):
        population_management = GenusManagement(compatibility=Compatibility(1))

        factory = IndividualFactory(BinaryRepresentation)
        agents: Agents = factory.create(3)
        agents.get(0).representation.genome = np.array([0, 0, 0, 0])
        agents.get(1).representation.genome = np.array([0, 0, 0, 0])
        agents.get(2).representation.genome = np.array([0, 2, 0, 0])
        population_management.initialize(agents)

        self.assertEqual(len(population_management.genus.species), 2)

    def test_speciation(self):
        population_management = GenusManagement(compatibility=Compatibility(1))

        factory = IndividualFactory(BinaryRepresentation)
        agents: Agents = factory.create(3)
        agents.get(0).representation.genome = np.array([0, 0, 0, 0])
        agents.get(1).representation.genome = np.array([0, 0, 0, 0])
        agents.get(2).representation.genome = np.array([0, 0, 0, 0])
        population_management.initialize(agents)

        self.assertEqual(len(population_management.genus.species), 1)

        population_management.genus.species[0].individuals.get(0).representation.genome = np.array([0, 2, 0, 0])
        population_management.speciate()

        self.assertEqual(len(population_management.genus.species), 2)
