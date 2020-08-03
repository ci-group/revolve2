from abc import abstractmethod
import math
import random

from nca.simulation import Measures


class Fitness:

    def __init__(self):
        self.fitness: float = 0.0

    @abstractmethod
    def calculate(self, measures: Measures):
        pass

    def __gt__(self, other):
        if isinstance(other, self.__class__):
            fitness = other.fitness
        else:
            fitness = other

        if self.fitness > fitness:
            return True
        else:
            return False

    def __ge__(self, other):
        if isinstance(other, self.__class__):
            fitness = other.fitness
        else:
            fitness = other

        if self.fitness >= fitness:
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
        # TODO fix
        self.fitness = measures.displacement


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self, measures: Measures):
        # process
        # TODO fix
        self.fitness = measures.rotation
