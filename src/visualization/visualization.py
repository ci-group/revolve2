from abc import abstractmethod
from typing import List

import matplotlib.pyplot as plt
import numpy as np

from visualization.analysis.statistics import Statistics


class Visualization:

    @abstractmethod
    def visualize(self):
        pass


class StatisticsVisualization:

    def __init__(self, title: str = "Statistics", x_axis: str = None, y_axis: str = None):
        self.title = title

        self.x_axis = x_axis
        self.y_axis = y_axis

        self.statistics = Statistics()

    @abstractmethod
    def prepare(self):
        pass

    def set_statistics(self, values):
        number_of_quartiles = 5
        labels = ['min', 'first', 'median', 'third', 'max']

        value_dictionary = {labels[index]: np.quantile(values, percentile / (number_of_quartiles-1))
                for index, percentile in enumerate(range(number_of_quartiles))}

        self.statistics.log(value_dictionary['max'], value_dictionary['third'], value_dictionary['median'],
                            value_dictionary['first'], value_dictionary['min'])

    def visualize(self):
        plt.plot(self.statistics.maximum_value)
        plt.plot(self.statistics.upper_quartile)
        plt.plot(self.statistics.median_value)
        plt.plot(self.statistics.lower_quartile)
        plt.plot(self.statistics.minimum_value)

        plt.title(self.title)

        if self.x_axis is not None:
            plt.xlabel(self.x_axis)

        if self.y_axis is not None:
            plt.ylabel(self.y_axis)

        plt.legend(["Maximum", "Upper Quartile", "Median", "Lower Quartile", 'Minimum'])
        plt.show()
