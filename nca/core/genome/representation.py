from abc import abstractmethod
from typing import List

import numpy as np

from nca.core.evolution.conditions.initialization import Initialization
from nca.experiment.configurations import RepresentationConfiguration

Genome = np.array


class Representation:

    def __init__(self):
        self.configuration = RepresentationConfiguration()
        self.genome: Genome = Genome([])

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
        self.genome[indexes] = self.genome[indexes[::-1]]