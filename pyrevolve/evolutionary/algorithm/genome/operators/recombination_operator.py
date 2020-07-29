from abc import abstractmethod

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.genome import Genome
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.shared.configurations import RecombinationConfiguration


class RecombinationOperator:

    def __init__(self):
        self.configuration = RecombinationConfiguration()
        pass


    def algorithm(self, representation_1: Representation, representation_2: Representation):
        # check if we do not have to do the mutation
        if self.configuration.recombination_probability < np.random.random():
            return representation_1, representation_2

        recombined_representation_1 = np.copy.deepcopy(representation_1)
        recombined_representation_2 = np.copy.deepcopy(representation_2)
        self._execute(recombined_representation_1, recombined_representation_2)
        return recombined_representation_1, recombined_representation_2

    @abstractmethod
    def _execute(self, representation_1: Representation, representation_2: Representation):
        pass


class OnePointCrossover(RecombinationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation_1: Representation, representation_2: Representation):
        crossover_index = representation_1.selection_indexes(k=1)[0]

        if crossover_index > round(representation_1.configuration.genome_size / 2):
            crossover_index = crossover_index - representation_1.configuration.genome_size

        print(crossover_index, representation_1.configuration.genome_size)

        tmp = representation_2.genome[:crossover_index].copy()
        representation_2.genome[:crossover_index], representation_1.genome[:crossover_index] = \
            representation_1.genome[:crossover_index], tmp

        return representation_1, representation_2
