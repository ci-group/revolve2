from abc import abstractmethod
from typing import List

import numpy as np

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.evolution.conditions.initialization import Initialization

Genome: List = []


class Representation:

    def __init__(self):
        self.configuration = RepresentationConfiguration()
        self.genome: Genome = []
        self.init()

    def init(self, initialization: Initialization = None):
        if initialization is not None:
            self.genome = initialization.algorithm(self.configuration.genome_size)
        else:
            self._initialize()

    @abstractmethod
    def _initialize(self):
        pass

    @abstractmethod
    def compatibility(self, other) -> float:
        pass

    def selection_indexes(self, k=2) -> List[int]:
        return np.random.choice(len(self.genome), k, replace=False)

    def range_selection(self) -> List[int]:
        indexes = sorted(self.selection_indexes())
        return list(range(indexes[0], indexes[1] + 1))

    def swap_indexes(self, indexes: List[int]):
        index1 = indexes[0]
        index2 = indexes[1]
        self.genome[index1], self.genome[index2] = self.genome[index2], self.genome[index1]

    @abstractmethod
    def compatibility(self, other) -> float:
        pass

    @abstractmethod
    def visit(self, representation_visitor):
        pass