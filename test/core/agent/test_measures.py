import unittest
from typing import List

from revolve.evosphere.performance_measures import PerformanceMeasures


class TestPerformanceMeasures(unittest.TestCase):

    def test_compare(self):

        measures_1 = PerformanceMeasures()
        measures_1.test()

        measures_2 = PerformanceMeasures()
        measures_2.test()

        measures: List[PerformanceMeasures] = [measures_1, measures_2]

        measures_sum = PerformanceMeasures.normalize(measures)

        for (key, value) in measures_sum.measures.items():
            self.assertEqual((measures_1.measures[key] + measures_2.measures[key]) / 2, value)
