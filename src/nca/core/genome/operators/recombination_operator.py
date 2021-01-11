import copy
import random
from abc import abstractmethod
from itertools import combinations
from random import shuffle
from typing import List

import numpy as np

from nca.core.abstract.configurations import OperatorConfiguration
from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from nca.core.genome.representations.representation import Representation


class RecombinationOperator:

    def __init__(self):
        self.configuration = OperatorConfiguration()
        pass

    def __call__(self, individuals: List[Individual], debug=False) -> Genotype:
        # avoid overwriting the original parents, shuffle for randomly selecting base genotype later
        genotypes = copy.deepcopy([individual.genotype for individual in individuals])  # takes a long time
        shuffle(genotypes)

        # Create a random ordering of parent combinations of only one representation in pairs of twos.
        parent_combinations = combinations(genotypes, 2)

        for (genotype_1, genotype_2) in parent_combinations:
            # check if we do NOT have to do the mutation, then continue to the next.
            if self.configuration.recombination_probability <= np.random.random() and not debug:
                continue

            representation_1, representation_2 = genotype_1.get_random_representations(genotype_2)
            self._recombine(representation_1, representation_2)

        # Get the genotype as the basis of recombinations
        return random.choice(genotypes)

    @abstractmethod
    def _recombine(self, representation_1: Representation, representation_2: Representation):
        pass


class NoCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation, representation_2: Representation):
        pass



class UniqueElementCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation,
                   representation_2: Representation):
        size_1 = len(representation_1)
        size_2 = len(representation_2)

        all_elements = copy.copy(representation_1)
        all_elements.extend(representation_2)

        # Remove duplicates
        unique_elements = list(dict.fromkeys(all_elements))

        representation_1[:] = np.random.choice(unique_elements, size_1, replace=False)
        representation_2[:] = np.random.choice(unique_elements, size_2, replace=False)


class OnePointCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation, representation_2: Representation):
        crossover_index = representation_1.random_index(offset=1)  # crossover has n-1 options.
        temp = copy.deepcopy(representation_1[crossover_index+1:])
        representation_1[crossover_index+1:] = representation_2[crossover_index+1:]
        representation_2[crossover_index+1:] = temp


class OnePointUniformCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation, representation_2: Representation):
        new_representation: Representation = random.choice([representation_1, representation_2])
        crossover_index = new_representation.random_index(offset=1)  # crossover has n-1 options.

        for index in range(crossover_index+1, len(new_representation)):
            new_representation[index] = (representation_1[index] + representation_2[index]) / 2


class OneElementCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation, representation_2: Representation):
        new_representation = random.choice([representation_1, representation_2])

        random_index = random.randint(0, len(new_representation) - 1)
        new_representation[random_index] = (representation_1[random_index] + representation_2[random_index]) / 2


class AllElementCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation, representation_2: Representation):
        new_representation = random.choice([representation_1, representation_2])

        for index in range(len(new_representation)):
            new_representation[index] = (representation_1[index] + representation_2[index]) / 2


class UniformCrossover(RecombinationOperator):

    def _recombine(self, representation_1: Representation, representation_2: Representation):
        # Reverse the order of the tuples to keep the base representation random.
        if np.random.random() > 0.5:
            # TODO mmmhhh...
            temp = copy.copy(representation_1)
            representation_1 = representation_2
            representation_2 = temp

        new_representation = representation_1

        for index, other_element in enumerate(representation_2):
            if random.random() > 0.5:
                new_representation[index] = other_element
