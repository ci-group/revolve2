from typing import List

import numpy as np

from visualization.analysis.statistics import Statistics


class SummaryStatistics(Statistics):

    def __init__(self):
        super().__init__()

    def analyze(self, statistics_list: List[Statistics]):
        self.maximum_value.append(np.mean([statistics.maximum_value for statistics in statistics_list]))
        self.upper_quartile.append(np.mean([statistics.upper_quartile for statistics in statistics_list]))
        self.median_value.append(np.mean([statistics.median_value for statistics in statistics_list]))
        self.lower_quartile.append(np.mean([statistics.lower_quartile for statistics in statistics_list]))
        self.minimum_value.append(np.mean([statistics.minimum_value for statistics in statistics_list]))
        return self
