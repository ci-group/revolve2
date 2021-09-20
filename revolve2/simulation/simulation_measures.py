
from typing import Dict, List


class Measures:
    def __init__(self, measurements: Dict):
        self.measurements: Dict[str] = measurements

    def __getitem__(self, item: str):
        if item not in self.measurements:
            return None

        return self.measurements[item]

    def __setitem__(self, key, value):
        self.measurements[key] = value

    def values(self):
        return list(self.measurements.values())

    def keys(self):
        return list(self.measurements.keys())


class SimulationMeasures(Measures):

    def __init__(self):
        self.categories = ['timestep', 'x', 'y', 'angle', 'energy']
        super().__init__({element: [] for element in self.categories})
        self.iterations = 0
        self.fitness = 0.0

    def add(self, simulation_iteration: List[float]):
        assert(len(simulation_iteration) == len(self.categories))

        for index, category in self.categories:
            self[category] = simulation_iteration[index]
        self.iterations += 1
