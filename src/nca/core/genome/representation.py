import collections
import random
from abc import abstractmethod, ABC
from typing import List, Tuple, Dict

import numpy as np

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.evolution.conditions.initialization import Initialization


class Representation(collections.MutableSequence, ABC):

    def __init__(self, initialization: Initialization):
        self.genome = []
        self.configuration = RepresentationConfiguration()
        self.initialization: Initialization = initialization

        if self.initialization is not None:
            self.genome.extend(self.initialization(self.configuration.genome_size))

    def random_value(self):
        return self.initialization(1)[0]

    @abstractmethod
    def compatibility(self, other) -> float:
        pass

    @abstractmethod
    def visit(self, representation_visitor):
        pass

    def selection_indexes(self, k=2) -> List[int]:
        return np.random.choice(len(self), k, replace=False)

    def range_selection(self) -> List[int]:
        indexes = sorted(self.selection_indexes(k=2))
        return list(range(indexes[0], indexes[1] + 1))

    def swap_indexes(self, indexes: Tuple[int]):
        self[indexes[0]], self[indexes[1]] = self[indexes[1]], self[indexes[0]]

    def invert_indexes(self, indexes: List[int]):
        self[indexes] = self[reversed(indexes)]

    def random_index(self, offset: int = 0):
        return random.choice(range(len(self) - offset))

    def __repr__(self):
        return str(self.genome)

    def __getitem__(self, item):
        return self.genome.__getitem__(item)

    def __delitem__(self, item):
        self.genome.__delitem__(item)

    def __len__(self):
        return self.genome.__len__()

    def __setitem__(self, key, value):
        self.genome.__setitem__(key, value)

    def insert(self, index: int, object: object) -> None:
        self.genome.insert(index, object)


class MultiRepresentation(Representation, ABC):

    def __init__(self, dictionary: Dict):
        self.genome = dictionary


class BrainRepresentation(Representation):
    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass


class MorphologyRepresentation(Representation):
    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass
