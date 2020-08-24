import math
import numpy as np

from nca.core.agent.fitness import Fitness, MultiFitness
from nca.core.agent.individual import Individual


class DisplacementFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual: Individual):
        delta_x = individual.measures['displacement_x'][-1] - individual.measures['displacement_x'][0]
        delta_y = individual.measures['displacement_y'][-1] - individual.measures['displacement_y'][0]
        return Fitness(math.sqrt(delta_x**2 + delta_y**2))


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual: Individual):
        return Fitness(np.sum(individual.measures['rotation']))


class DisplacementRotationalFitness(MultiFitness):

    def __init__(self):
        super().__init__(fitnesses=[DisplacementFitness(), RotationalFitness()])


class OnesFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual: Individual):
        number_of_elements = len(individual.representation.genome)
        difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(individual.representation.genome)))
        return Fitness(number_of_elements - difference)
