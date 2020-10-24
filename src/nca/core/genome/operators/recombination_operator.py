import copy
import random
from abc import abstractmethod
from itertools import combinations
from random import shuffle
from typing import List, Tuple, Dict

import numpy as np

from nca.core.abstract.configurations import OperatorConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.genome.representation import Representation

class RecombinationOperator:

    def __init__(self):
        self.configuration = OperatorConfiguration()
        pass

    def __call__(self, parents: Actors) -> Dict[str, Representation]:
        # TODO refactoring of whole recombination
        parents = copy.deepcopy(parents) # avoid overwriting the original parents.
        parent_genotype = [parent.genotype for parent in parents]
        number_of_parents = len(parents)
        assert(number_of_parents > 0)

        # Create a random ordering of parent combinations of only one representation in pairs of twos.
        parent_combinations: List[List[Dict[str, Representation]]] = \
            [list(representations_combination) for representations_combination in combinations(parent_genotype, 2)]

        shuffle(parent_combinations)

        new_genotype = parents[0].genotype

        # check if we do have to do the mutation # n * (n-1) / 2 = 1 + 2 + 3 + ... + n-1
        for parent_combination in parent_combinations: #range(int(number_of_parents * (number_of_parents - 1) / 2)):
            if self.configuration.recombination_probability > np.random.random():
                selected_key = np.random.choice(list(parent_combination[0].keys()))
                parent_representations = []

                for parent in parent_combination:
                    parent_representations.append(parent[selected_key])

                new_genotype[selected_key] = self._recombine(parent_representations)

        return new_genotype

    @abstractmethod
    def _recombine(self, representations: List[Representation]) -> Representation:
        pass

    def compatibility(self, initialization_type):
        return True


class OnePointCrossover(RecombinationOperator):

    def _recombine(self, representations: List[Representation]) -> Representation:
        # Reverse the order of the tuples to keep the base representation random.
        if np.random.random() > 0.5:
            representations[:] = representations[::-1]

        crossover_index = representations[0].random_index(offset=1)  # crossover has n-1 options.

        temp = copy.deepcopy(representations[0][crossover_index+1:])
        representations[0][crossover_index+1:] = representations[1][crossover_index+1:]
        representations[1][crossover_index+1:] = temp

        return representations[0]


class OnePointUniformCrossover(RecombinationOperator):

    def _recombine(self, representations: List[Representation]) -> Representation:
        # Reverse the order of the tuples to keep the base representation random.
        if np.random.random() > 0.5:
            representations = representations[::-1]

        new_representation: Representation = representations[0]
        crossover_index = new_representation.random_index(offset=1)  # crossover has n-1 options.

        for index in range(crossover_index+1, len(new_representation)):
            new_representation[index] = (new_representation[index] + representations[1][index]) / 2

        return new_representation


class OneElementCrossover(RecombinationOperator):

    def _recombine(self, representations: List[Representation]) -> Representation:
        # Reverse the order of the tuples to keep the base representation random.
        if np.random.random() > 0.5:
            representations = representations[::-1]

        new_representation = representations[0]

        random_index = random.randint(0, len(representations[0])-1)
        new_representation[random_index] = (new_representation[random_index] + representations[1][random_index]) / 2

        return new_representation


class AllElementCrossover(RecombinationOperator):

    def _recombine(self, representations: List[Representation]) -> Representation:
        # Reverse the order of the tuples to keep the base representation random.
        new_representation = representations[0]
        for index in range(len(new_representation)):
            new_representation[index] = (new_representation[index] + representations[1][index]) / 2
        return new_representation


class UniformCrossover(RecombinationOperator):

    def _recombine(self, representations: List[Representation]) -> Representation:
        # Reverse the order of the tuples to keep the base representation random.
        if np.random.random() > 0.5:
            representations = representations[::-1]

        new_representation = representations[0]
        for index, other_element in enumerate(representations[1]):
            if random.random() > 0.5:
                new_representation[index] = other_element

        return new_representation
