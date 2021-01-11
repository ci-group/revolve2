import numpy as np

from nca.core.actor.individual import Individual


class FitnessEvaluation(object):
    pass


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
        return [difference, individual.id, ]

