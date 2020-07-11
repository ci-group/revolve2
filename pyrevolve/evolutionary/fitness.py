import math
from abc import ABC, abstractmethod

from pyrevolve.evolutionary.things.performance_measures import PerformanceMeasures


class Fitness(ABC):

    def __init__(self):
        self.measures: PerformanceMeasures = PerformanceMeasures()
        self.fitness: float = 0.0

    def process(self, performance_measures: PerformanceMeasures):
        self.measures = performance_measures
        self.fitness = self.calculate()

    @abstractmethod
    def calculate(self):
        pass

    @staticmethod
    def best():
        fitness = Fitness()
        fitness.fitness = math.inf
        return fitness

    @staticmethod
    def worst():
        fitness = Fitness()
        fitness.fitness = -math.inf
        return fitness

    def __gt__(self, other):
        if self.fitness > other.fitness:
            return True
        else:
            return False


class TestFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self):
        pass


class DisplacementFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self):
        return self.measures.displacement


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self):
        return self.measures.rotation
