from abc import abstractmethod
from typing import List

import numpy as np

from pyrevolve.evolutionary import Agents
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.shared.configurations import RecombinationConfiguration


class RecombinationOperator:

    def __init__(self):
        self.configuration = RecombinationConfiguration()
        pass

    def algorithm(self, parents: Agents):
        # check if we do not have to do the mutation
        if self.configuration.recombination_probability < np.random.random():
            return

        self._execute([parent.representation for parent in parents])

    @abstractmethod
    def _execute(self, representations: List[Representation]):
        pass


class OnePointCrossover(RecombinationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representations: List[Representation]):
        # TODO make possible to do multiple parents
        crossover_index = representations[0].selection_indexes(k=1)[0]

        if crossover_index > round(representations[0].configuration.genome_size / 2):
            crossover_index = crossover_index - representations[0].configuration.genome_size

        tmp = representations[1].genome[:crossover_index].copy()
        representations[1].genome[:crossover_index], representations[0].genome[:crossover_index] = \
            representations[0].genome[:crossover_index], tmp

        return representations[0], representations[1]
