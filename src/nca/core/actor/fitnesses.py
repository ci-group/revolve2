import math
import numpy as np

from nca.core.actor.fitness import Fitness, MultiFitness
from revolve.robot import robot


class DisplacementFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, robot):
        delta_x = robot.measures['displacement_x'][-1] - robot.measures['displacement_x'][0]
        delta_y = robot.measures['displacement_y'][-1] - robot.measures['displacement_y'][0]
        return Fitness(math.sqrt(delta_x**2 + delta_y**2))


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, robot):
        return Fitness(np.sum(robot.measures['rotation']))


class DisplacementRotationalFitness(MultiFitness):

    def __init__(self):
        super().__init__(fitnesses=[DisplacementFitness(), RotationalFitness()])


class OnesFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual):
        number_of_elements = len(individual.representation)
        difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(individual.representation)))
        return Fitness(number_of_elements - difference)
