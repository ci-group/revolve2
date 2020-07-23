import math
from abc import ABC, abstractmethod
import random
from pyrevolve.evolutionary.things.performance_measures import PerformanceMeasures
from pyrevolve.simulator.measures import Measures


class Fitness(ABC):

    def __init__(self):
        self.fitness: float = 0.0

    @abstractmethod
    def calculate(self, measures: Measures):
        pass

    def __get__(self, instance, owner) -> float:
        return self.fitness

    def __gt__(self, other):
        if self.fitness > other.fitness:
            return True
        else:
            return False

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


class TestFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self, measures: Measures):
        self.fitness = random.random()


class DisplacementFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self, measures: Measures):
        # process
        self.fitness = measures.displacement


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self, measures: Measures):
        # process
        self.fitness = measures.rotation
