import numpy as np

from abc import abstractmethod
from typing import List


class Fitness(float):

    def __init__(self, initial_value: float = 0.0):
        float.__init__(initial_value)

    @abstractmethod
    def __call__(self, individual):
        pass

    def __new__(cls, initial_value: float = 0.0):
        return super().__new__(cls, initial_value)


class MultiFitness(Fitness):

    def __new__(cls, fitnesses: List[Fitness] = None, initial_value: float = 0.0):
        return super().__new__(cls, initial_value)

    def __init__(self, fitnesses: List[Fitness], initial_value: float = 0.0):
        super().__init__(initial_value)
        self.fitnesses: List[Fitness] = fitnesses

    def __call__(self, robot):
        for index, fitness in enumerate(self.fitnesses):
            self.fitnesses[index] = fitness(robot)
        return MultiFitness(self.fitnesses, float(np.mean(self.fitnesses)))


class CombinedFitness(Fitness):

    def __new__(cls, fitnesses: List[Fitness] = None, initial_value: float = 0.0):
        return super().__new__(cls, initial_value)

    def __init__(self, fitnesses: List[Fitness] = None, initial_value: float = 0.0):
        super().__init__(initial_value)
        self.fitnesses: List[Fitness] = fitnesses if fitnesses is not None else []

    def add(self, fitness: Fitness):
        self.fitnesses.append(fitness)

    def __call__(self):
        return CombinedFitness(fitnesses=self.fitnesses, initial_value=float(np.mean(self.fitnesses)))
