import math

import numpy as np

from simulation.simulation_measures import Measures, SimulationMeasures


class PerformanceMeasures(Measures):

    def __init__(self):
        self.categories = ['time_duration', 'displacement_x', 'displacement_y', 'rotation', 'energy']
        super().__init__({element: 0.0 for element in self.categories})

        self.distance = 0.0
        self.velocity = 0.0

    def process(self, measures: SimulationMeasures):
        for index, category in enumerate(measures.categories):
            self[self.categories[index]] = measures['x'][-1] - measures[category][0]

        self.distance = math.sqrt(self['displacement_x']**2 + self['displacement_y']**2)
        velocity_x = self['displacement_x'] / self['time_duration']
        velocity_y = self['displacement_y'] / self['time_duration']
        self.velocity = math.sqrt(velocity_x**2 + velocity_y**2)
