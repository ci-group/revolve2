import unittest

from revolve2.nca.core.actor.fitness_evaluation import OnesNSGAFitness
from revolve2.nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration
from revolve2.nca.core.evolution.selection.survival_non_dominated import NonDominatedSortingSurvival
from revolve2.nca.evolution import Evolution


class TestEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution(evolutionary_configuration=GeneticAlgorithmConfiguration(survivor_selection=NonDominatedSortingSurvival(minimization=False, debug=False)), fitness_evaluation=OnesNSGAFitness())
        evolution.evolve()
        self.assertTrue(True)
