import math
import numpy as np

from nca.core.actor.fitness_evaluation import FitnessEvaluation
from nca.core.actor.individual import Individual
from revolve.robot.robot import Robot


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
