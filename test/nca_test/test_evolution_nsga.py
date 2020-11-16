import unittest

from nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration
from nca.core.evolution.selection.survival_non_dominated import NonDominatedSortingSurvival
from nca.evolution import Evolution


class TestEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution(evolutionary_configuration=GeneticAlgorithmConfiguration(survivor_selection=NonDominatedSortingSurvival()))
        evolution.evolve()
        self.assertTrue(True)
