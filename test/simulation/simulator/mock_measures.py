import numpy as np

from revolve.evosphere.performance_measures import PerformanceMeasures
from simulation.simulation_measures import SimulationMeasures


class MockSimulationMeasures(SimulationMeasures):

    def __init__(self):
        super().__init__()
        for key in self.measurements.keys():
            self[key] = np.cumsum(np.random.uniform(0.0, 1.0, 10))


class MockPerformanceMeasures(PerformanceMeasures):

    def __init__(self):
        super().__init__()
        for key in self.measurements.keys():
            self[key] = np.cumsum(np.random.uniform(0.0, 1.0, 10))
