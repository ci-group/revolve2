import random
from abc import abstractmethod
from typing import List

import numpy as np

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.evolution.conditions.initialization import Initialization


Genome: List = []


class Representation:

    def __init__(self, initialization):
        self.configuration = RepresentationConfiguration()
        self.initialization = initialization
        self.genome: Genome = []

        if self.initialization is not None:
            self.initialize()

    def initialize(self):
        self.genome = self.initialization(self.configuration.genome_size)

    def random_value(self):
        return self.initialization(1)[0]

    @abstractmethod
    def compatibility(self, other) -> float:
        pass

    @abstractmethod
    def visit(self, representation_visitor):
        pass

    def selection_indexes(self, k=2) -> List[int]:
        return np.random.choice(self.__len__(), k, replace=False)

    def range_selection(self) -> List[int]:
        indexes = sorted(self.selection_indexes())
        return list(range(indexes[0], indexes[1] + 1))

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
