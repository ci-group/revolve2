import unittest

from nca.core.analysis.statistics_plotter import plot_statistics
from nca.evolution import Evolution
from nca.repeated_evolution import RepeatedEvolution


class TestRepeatedEvolution(unittest.TestCase):

    def test_create(self):
        evolution = RepeatedEvolution()
        summary = evolution.evolve()
        #plot_statistics(summary[0])
        self.assertTrue(True)

    def test_evolution(self):
        evolution = Evolution()
        summary = evolution.evolve()
        #plot_statistics(summary[0])
        self.assertTrue(True)
