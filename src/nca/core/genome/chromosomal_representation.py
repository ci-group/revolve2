import random
from typing import List

import numpy as np

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.genome.representation import Representation, Genome


class ChromosomalRepresentation(Representation):

    def __init__(self, initialization):
        self.configuration = RepresentationConfiguration()
        self.initialization = initialization
        self.genome: List[Genome] = []

    def initialize(self):
        self.genome: List[Genome] = []
        for _ in range(self.configuration.number_of_chromosomes):
            self.genome.append(self.initialization(self.configuration.genome_size))

    def selection_indexes(self, k=2) -> List[List[int]]:
        tuples = []
        for chromosome_index in range(self.configuration.number_of_chromosomes):
            [tuples.append([chromosome_index, value]) for value in np.random.choice(len(self.genome[chromosome_index]), k, replace=False)]
        return tuples

    def range_selection(self) -> List[int]:
        #todo
        pass

    def swap_indexes(self, indexes: List[int]):
        index1 = indexes[0]
        index2 = indexes[1]
        self.genome[index1], self.genome[index2] = self.genome[index2], self.genome[index1]

    def __str__(self):
        return str(self.genome)

    def __len__(self):
        return len(self.genome)

    def random_index(self):
        return random.choice(range(len(self.genome)))
