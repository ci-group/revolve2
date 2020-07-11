import random
import string
from typing import Dict, List


class PerformanceMeasures:

    def __init__(self):
        self.measures: Dict[string, float] = {'velocity': 0.0, 'displacement': 0.0, 'rotation': 0.0, 'balance': 0.0}

    def test(self):
        for key in self.measures.keys():
            self.measures[key] = random.uniform(0.0, 1.0)

    @staticmethod   # TODO List typing
    def normalize(measures: List):
        performance_measures: PerformanceMeasures = PerformanceMeasures()

        # add_measures
        for performance_measure in measures:
            for (key, value) in performance_measure.measures.items():
                performance_measures.measures[key] += value

        # normalize measures
        for key in performance_measures.measures.keys():
            performance_measures[key] /= len(measures)

        return performance_measures
