import copy
from abc import abstractmethod
from typing import List

import numpy as np

from nca.core.agent.agents import Agents
from nca.core.genome.representation import Representation
from nca.core.abstract.configurations import RecombinationConfiguration


class RecombinationOperator:

    def __init__(self):
        self.configuration = RecombinationConfiguration()
        pass

    def algorithm(self, parents: Agents) -> Representation:
        assert(len(parents) > 0)

        # check if we do not have to do the mutation
        if self.configuration.recombination_probability < np.random.random():
            return parents[0].representation

        return self._execute([parent.representation for parent in parents])

    @abstractmethod
    def _execute(self, representations: List[Representation]) -> Representation:
        pass


class OnePointCrossover(RecombinationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representations: List[Representation]) -> Representation:
        new_representation: Representation = copy.deepcopy(representations[0])

        # TODO make possible to do multiple parents
        crossover_index = np.random.choice(len(new_representation.genome) - 1, 1, replace=False)[0] + 1

        if crossover_index > round(representations[0].configuration.genome_size / 2):
            crossover_index = crossover_index - representations[0].configuration.genome_size

        new_representation.genome[:crossover_index] = representations[1].genome[:crossover_index].copy()

        return new_representation
