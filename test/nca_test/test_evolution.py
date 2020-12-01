import unittest

from nca.core.actor.individual_factory import IndividualFactory
from nca.core.genome.representations.chromosomal_representation import ChromosomalRepresentation
from nca.core.genome.initialization import IntegerInitialization
from nca.evolution import Evolution


class TestEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution()
        evolution.evolve()
        self.assertTrue(True)

    def test_chromosomes(self):
        evolution = Evolution(individual_factory=IndividualFactory(representation_type=ChromosomalRepresentation,
                                                                   initialization_type=IntegerInitialization))
        evolution.evolve()
        self.assertTrue(True)
