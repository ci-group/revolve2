import math
from abc import abstractmethod
import numpy as np

from nca.core.agent.individual import Individual
from nca.core.genome.representation import Representation


class Fitness:

    def __init__(self):
        pass

    @abstractmethod
    def __call__(self, individual: Individual):
        pass
    """
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
    """


class DisplacementFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual: Individual) -> float:
        delta_x = individual.measures['x'][-1] - individual.measures['x'][0]
        delta_y = individual.measures['y'][-1] - individual.measures['y'][0]
        return math.sqrt(delta_x**2 + delta_y**2)


class RotationalFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual: Individual):
        return np.sum(individual.measures['rotation'])


class OnesFitness(Fitness):

    def __init__(self):
        super().__init__()

    def __call__(self, individual: Individual) -> float:
        number_of_elements = len(individual.representation.genome)
        ones = np.ones(number_of_elements)
        return number_of_elements - np.sum(np.abs(ones - np.array(individual.representation.genome)))


if __name__ == "__main__":

    arrays = [[1.019429889891763, 0.8238184647852573, 0.5928976536600072, 1.4504746408677665, 0.4962040046430225, 1.2972219698811729, 1.0189531385035826, 1.5045625664354771, 1.3740525100717729, 0.55071591855098],
              [1.019429889891763, 0.8238184647852573, 0.5928976536600072, 1.4504746408677665, 0.4962040046430225, 1.2972219698811729, 1.0189531385035826, 1.5045625664354771, 1.3740525100717729, 0.55071591855098],
              [0.9475531404611999, 0.3745616245842027, 0.3077748489483123, 0.8171641775229619, 1.3683297766643114, 0.8705334809172709, 1.4489382166181088, 1.146375797998035, 0.8906615095363624, 0.7778943995639207],
              [1.0429091954740162, 0.448129128008393, 0.7765073540125769, 1.1618702135045067, 1.000494592772392, 1.3727541480326566, 0.37405832275596596, 1.1065134125540212, 1.3589964217172703, 1.3635965219225645]]

    for arr in arrays:
        rep = Representation()
        rep.genome = arr
        ind = Individual(rep)
        fitness = OnesFitness()(ind)
        print(fitness)
