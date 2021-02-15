from abc import abstractmethod
from typing import List, Dict

import matplotlib.pyplot as plt
import numpy as np

from visualization.analysis.statistics import Statistics


class Visualization:

    @abstractmethod
    def visualize(self):
        pass


class TimeSeriesVisualization:

    def __init__(self, values: Dict[str, List]):
        self.values: Dict[str, List] = values

    def visualize(self):
        for value_series_key in self.values.keys():
            plt.plot(self.values[value_series_key])
        plt.legend(self.values.keys())
        plt.show()


def time_series_visualization(values: Dict[str, List]):
    ts_visualization = TimeSeriesVisualization(values)
    ts_visualization.visualize()


class StatisticsVisualization:

    def __init__(self, title: str = "Statistics", x_axis: str = None, y_axis: str = None):
        self.title = title

        self.x_axis = x_axis
        self.y_axis = y_axis

        self.statistics = Statistics()

    @abstractmethod
    def prepare(self):
        pass

    def visualize(self, show=False):
        plt.plot(self.statistics.maximum_value, color="red", alpha=0.75)
        #plt.plot(self.statistics.upper_quartile)
        plt.plot(self.statistics.median_value, color="green", alpha=0.5)
        #plt.plot(self.statistics.lower_quartile)
        plt.plot(self.statistics.minimum_value, color="blue", alpha=0.75)

        plt.grid(b=True, which='major', color='#666666', linestyle='-', alpha=0.5)

        # Show the minor grid lines with very faint and almost transparent grey lines
        plt.minorticks_on()
        plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.25)

        plt.title(self.title)

        if self.x_axis is not None:
            plt.xlabel(self.x_axis)

        if self.y_axis is not None:
            plt.ylabel(self.y_axis)

        plt.fill_between(range(len(self.statistics.lower_quartile)),
                         self.statistics.lower_quartile, self.statistics.upper_quartile, alpha=0.5, color="khaki")
        plt.legend(["Maximum", "Median", 'Minimum', 'IQR'])
        if show:
            plt.show()

        plt.close()
