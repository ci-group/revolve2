import copy
import math
from abc import abstractmethod

import numpy as np

from nca.core.actor.individual import Individual
from revolve.robot.robot import Robot


class FitnessEvaluation:

    def __init__(self, multi_fitness: bool = False):
        self.multi_fitness = multi_fitness

    @abstractmethod
    def calculate(self, individual: Individual):
        pass

    def __call__(self, individual: Individual):
        if self.multi_fitness:
            self.calculate(individual)
        else:
            individual.fitness.add(self.__class__.__name__, self.calculate(individual))

        return individual.fitness.value()


class DisplacementFitness(FitnessEvaluation):

    def calculate(self, robot: Robot):
        delta_x = robot.measures['displacement_x'][-1] - robot.measures['displacement_x'][0]
        delta_y = robot.measures['displacement_y'][-1] - robot.measures['displacement_y'][0]
        return math.sqrt(delta_x**2 + delta_y**2)


class RotationalFitness(FitnessEvaluation):

    def calculate(self, robot: Robot):
        return np.sum(robot.measures['rotation'])


class DisplacementRotationalFitness(FitnessEvaluation):

    def __init__(self):
        super().__init__(multi_fitness=True)
        self.fitnesses = [DisplacementFitness(), RotationalFitness()]

    def calculate(self, individual: Individual):
        fitnesses = {}
        for fitness in self.fitnesses:
            fitnesses[fitness.__class__.__name__] = fitness(individual)
        return fitnesses


class OnesFitness(FitnessEvaluation):

    def calculate(self, individual: Individual):
        representation = individual.get_representation()
        number_of_elements = len(representation)

        difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(representation)))
        return -difference


class OnesNSGAFitness(FitnessEvaluation):

    def calculate(self, individual: Individual):
        representation = individual.get_representation()
        number_of_elements = len(representation)
        difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(representation)))
        return [difference, individual.id, ]
