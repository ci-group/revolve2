from abc import ABC, abstractmethod
from typing import List

import numpy as np

from revolve2.nca.core.actor.individual import Individual


class FitnessEvaluation(ABC):

    @abstractmethod
    def __call__(self, individual: Individual):
        pass


class FitnessEvaluations(FitnessEvaluation):

    def __init__(self, fitnesses: List[FitnessEvaluation]):
        self.fitnesses = fitnesses

    def __call__(self, individual: Individual):
        fitnesses = {}
        for fitness in self.fitnesses:
            fitnesses[fitness.__class__.__name__] = fitness(individual)
        return fitnesses


class OnesFitness(FitnessEvaluation):

    def __call__(self, individual: Individual):
        representation = individual.get_representation()
        number_of_elements = len(representation)

        difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(representation)))
        return -difference


class OnesNSGAFitness(FitnessEvaluation):

    def __call__(self, individual: Individual):
        representation = individual.get_representation()
        number_of_elements = len(representation)
        difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(representation)))
        return {"objectiveA": difference, "objeciveB": individual.id, }
