"""
import random
from typing import List

import numpy as np

from revolve2.abstract.configurations import RepresentationConfiguration
from revolve2.nca.core.genome.representation import Representation


class ChromosomalRepresentation(Representation):

    def __init__(self, initialization):
        self.configuration = RepresentationConfiguration()
        self.initialization = initialization
<<<<<<< Updated upstream:src/nca/core/genome/representations/chromosomal_representation.py
=======
        self.genome: List = []
        self.initialize()


    def initialize(self):
        for _ in range(self.configuration.number_of_chromosomes):
            self.append(self.initialization(self.configuration.genome_size))

    def selection_indexes(self, k=2) -> List[List]:
        tuples = []
        for chromosome_index in range(self.configuration.number_of_chromosomes):
            [tuples.append([chromosome_index, value])
             for value in np.random.choice(len(self[chromosome_index]), k, replace=False)]
        return tuples

    def range_selection(self) -> List:
        #todo
        pass

    def random_index(self, offset: int = 0):
        chromosome_index = random.choice(range(len(self.genome)))
        return [chromosome_index, random.choice(range(len(self.genome[chromosome_index]) - offset))]

    def swap_indexes(self, indexes: List):
        index1 = indexes[0]
        index2 = indexes[1]
        print("swap", index1, index2)
        self[index1], self[index2] = self[index2], self[index1]

<<<<<<< Updated upstream:src/nca/core/genome/representations/chromosomal_representation.py
    def random_index(self):
        return random.choice(range(len(self)))

=======
    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass

"""
