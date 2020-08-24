import unittest

from revolve.evosphere.performance_measures import PerformanceMeasures
from simulation_test.simulator.mock_measures import MockSimulationMeasures


class TestPerformanceMeasures(unittest.TestCase):

    def test_measures(self):

        simulation_measures = MockSimulationMeasures()
        performance_measures = PerformanceMeasures()

        performance_measures.process(simulation_measures)

        for category in performance_measures.categories:
            self.assertTrue(performance_measures[category] > 0.0)

        self.assertTrue(performance_measures.velocity > 0.0)
        self.assertTrue(performance_measures.distance > 0.0)
