import copy
from abc import abstractmethod
from itertools import combinations
from random import shuffle
from typing import List, Tuple

import numpy as np

from nca.core.abstract.configurations import OperatorConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.genome.representation import Representation


class RecombinationOperator:

    def __init__(self):
        self.configuration = OperatorConfiguration()
        pass

    def __call__(self, parents: Actors) -> Representation:
        parents = copy.deepcopy(parents) # avoid overwriting the original parents.
        number_of_parents = len(parents)
        assert(number_of_parents > 0)

        # Create a random ordering of parent combinations in pairs of twos.
        parent_combinations: List[Tuple[Individual, Individual]] = \
            [combination for combination in combinations(parents, 2)]
        shuffle(parent_combinations)

        new_representation = parents[0].representation  # Backup if no recombinations happen

        # check if we do have to do the mutation # n * (n-1) / 2 = 1 + 2 + 3 + ... + n-1
        for recombination_index in range(int(number_of_parents * (number_of_parents - 1) / 2)):
            if self.configuration.recombination_probability > np.random.random():
                new_representation = self._recombine(parent_combinations[recombination_index])

        return new_representation

    @abstractmethod
    def _recombine(self, representations: Tuple[Individual, Individual]) -> Representation:
        pass


class OnePointCrossover(RecombinationOperator):

    def _recombine(self, individuals: Tuple[Individual, Individual]) -> Representation:
        # Reverse the order of the tuples to keep the base representation random.
        if np.random.random() > 0.5:
            individuals = individuals[::-1]

        new_representation = individuals[0].representation
        crossover_index = new_representation.random_index(offset=1)  # crossover has n-1 options.

        #if crossover_index > round(representations[0].configuration.genome_size / 2):
        #    crossover_index = crossover_index - representations[0].configuration.genome_size

        temp = copy.deepcopy(new_representation[crossover_index+1:])
        new_representation[crossover_index+1:] = individuals[1].representation[crossover_index+1:]
        individuals[1].representation[crossover_index + 1:] = temp

        return new_representation
