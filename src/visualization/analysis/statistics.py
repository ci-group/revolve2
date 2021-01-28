from typing import List, Dict

import numpy as np


class Statistics:

    def __init__(self):
        self.maximum_value = []
        self.upper_quartile = []
        self.median_value = []
        self.lower_quartile = []
        self.minimum_value = []

    def log_quartiles(self, max, third, median, first, min):
        self.maximum_value.append(max)
        self.upper_quartile.append(third)
        self.median_value.append(median)
        self.lower_quartile.append(first)
        self.minimum_value.append(min)

    def log_values(self, values):
        number_of_quartiles = 5
        labels = ['min', 'first', 'median', 'third', 'max']
        value_dictionary = {labels[index]: np.quantile(values, percentile / (number_of_quartiles - 1))
                            for index, percentile in enumerate(range(number_of_quartiles))}

        self.log_quartiles(value_dictionary['max'], value_dictionary['third'], value_dictionary['median'],
                            value_dictionary['first'], value_dictionary['min'])

    def latest(self):
        return [self.maximum_value[-1],
                self.upper_quartile[-1],
                self.median_value[-1],
                self.lower_quartile[-1],
                self.minimum_value[-1]]

    def __repr__(self):
        return str(self.latest())


class MeasurementStatistics(Dict[str, Statistics]):

    def __init__(self, variables: List[str]):
        super().__init__()
        for variable in variables:
            self[variable] = Statistics()

    def latest(self):
        latest_dictionary = {}
        for key in self.keys():
            latest_dictionary[key] = self[key].latest()
        return latest_dictionary
