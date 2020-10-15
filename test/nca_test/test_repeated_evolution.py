import unittest

from nca.core.analysis.statistics_plotter import plot_statistics
from nca.repeated_evolution import RepeatedEvolution


class TestRepeatedEvolution(unittest.TestCase):

    def test_create(self):
        evolution = RepeatedEvolution()
        summary = evolution.evolve()
        plot_statistics(summary)
        self.assertTrue(True)

    def test_create(self):
        evolution = RepeatedEvolution()
        summary = evolution.evolve()
        plot_statistics(summary)
        self.assertTrue(True)