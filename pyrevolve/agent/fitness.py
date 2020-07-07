from abc import ABC
from typing import List


class Fitness(ABC):

    def __init__(self):
        self.measures: List[float] = []
        self.selected_fitness: float = 0.0

    def set_fitness(self, values: List[float], fitness_index: int = 0):
        self.measures = values
        self.selected_fitness = values[fitness_index]

    def fitness(self) -> float:
        return self.selected_fitness


class DisplacementFitness(Fitness):

    def __init__(self):
        super().__init__()


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()
