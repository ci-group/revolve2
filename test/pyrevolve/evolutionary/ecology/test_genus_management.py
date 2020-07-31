import math
import unittest

from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import BinaryRepresentation
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.ecology.speciation.compatibility import Compatibility
from pyrevolve.evolutionary.ecology.speciation.genus_management import GenusManagement
from pyrevolve.evolutionary.individual_factory import IndividualFactory


class GenusManagementTest(unittest.TestCase):

    def test_management(self):
        population_management = GenusManagement()

        agents1: Agents = IndividualFactory(BinaryRepresentation).create(n=3)
        population_management.initialize(agents1)

        for index in range(len(agents1)):
            self.assertEqual(population_management.genus.species[0].individuals[index], agents1[index])

        agents2: Agents = IndividualFactory(BinaryRepresentation).create(n=3)
        for agent in agents2:
            population_management.assign(agent)

        agents1.extend(agents2)
        for index in range(len(agents1)):
            self.assertEqual(population_management.genus.species[0].individuals[index], agents1[index])
