import math
import unittest

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import BinaryRepresentation
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.ecology.speciation.compatibility import Compatibility
from pyrevolve.evolutionary.ecology.speciation.genus import Genus
from pyrevolve.evolutionary.ecology.speciation.genus_management import GenusManagement
from pyrevolve.evolutionary.individual_factory import IndividualFactory


class GenusTest(unittest.TestCase):

    def test_add(self):
        genus = Genus(Compatibility(limit=math.inf))

        n = 3
        for _ in range(n):
            genus.add(Population(IndividualFactory().create(3)))

        self.assertEqual(len(genus.species), n)

    def test_insert(self):
        genus = Genus(Compatibility(limit=math.inf))

        n = 3
        factory = IndividualFactory(BinaryRepresentation)
        genus.add(Population(factory.create(n)))

        individual: Individual = factory.create(1)[0]
        genus.insert(individual)

        self.assertEqual(len(genus.species), 1)
        self.assertEqual(len(genus.species[0].individuals), n+1)
