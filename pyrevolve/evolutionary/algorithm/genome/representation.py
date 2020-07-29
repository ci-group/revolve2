import copy
from abc import abstractmethod
from typing import List

import numpy as np

from pyrevolve.evolutionary.algorithm.genome.genome import Genome
from pyrevolve.shared.configurations import RepresentationConfiguration


class Representation:

    def __init__(self):
        self.configuration = RepresentationConfiguration()
        self.genome: Genome = Genome([])

    def init(self, genome: Genome = None):
        if genome is not None:
            if self.genome.shape == genome.shape:
                self.genome = copy.deepcopy(genome)
            else:
                raise Exception("New genome does not have the same size as the current genome.")
        else:
            self._initialize()

    @abstractmethod
    def _initialize(self):
        pass

    @abstractmethod
    def compatibility(self, other):
        pass

    def selection_indexes(self, k=2) -> List[int]:
        return np.random.choice(len(self.genome), k, replace=False)

    def range_selection(self) -> List[int]:
        indexes = sorted(self.selection_indexes())
        return list(range(indexes[0], indexes[1] + 1))

    def swap_indexes(self, indexes: List[int]):
        self.genome[indexes] = self.genome[indexes[::-1]]