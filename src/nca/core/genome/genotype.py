from abc import abstractmethod, ABC
from typing import Dict, List

import numpy as np

from nca.core.genome.representations.representation import Representation


class Genotype(Dict[str, Representation]):

    def __init__(self, representations, keys=None):
        super().__init__()

        if representations is None:
            return

        #if isinstance(representations, Representation):
        self._initialize_single_representation(representations, keys)
        #else:
        #    self._initialize_multiple_representations(representations, keys)

    def initialize(self):
        for key in self.keys():
            self[key].initialize()

    def __call__(self, *args, **kwargs) -> Representation:
        pass

    # TODO consolidate with multiple representation initialization
    def _initialize_single_representation(self, representation: Representation, key=None):
        if key is None:
            key = str(type(representation).__name__)
        self[key] = representation

    def _initialize_multiple_representations(self, representations: List[Representation], keys: List[str] = None):
        for index, representation in enumerate(representations):
            if keys is None or len(keys) <= index:
                key = representation.__class__.__name__
                if key in self.keys():
                    raise Exception("Genotype: Cannot add multiple representations of the same type without specifying keys.")
            else:
                key = keys[index]
            self[key] = representation

    def compatibility(self, other):
        if len(self.keys()) == 1:
            key = list(self.keys())[0]
            return self[key].compatibility(other[key])

        sum: float = 0
        for key in self.keys():
            sum += self[key].compatibility(other[key])
        return sum

    def _recursively_get_representations(self, current: Dict):
        representations: List[Representation] = []
        # Recursively search for representations

        if isinstance(current, Representation):
            return [current]  # leaf node

        for key in current.keys():
            # Merge leaf nodes, if any
            representations.extend(self._recursively_get_representations(current[key]))

        # Return current list of leaf nodes
        return representations

    def representations(self) -> List[Representation]:
        return self._recursively_get_representations(self)

    def get_random_representation(self, key=None):
        # Select random key if not chosen, otherwise check that the key is in the genotype.
        if key is None:
            key = np.random.choice(list(self.keys()))
        else:
            if key not in self:
                raise Exception("Wrong key for getting a representation")

        representation = self[key]

        # Continue searching in the dictionary chain of keys until we find a key that leads to a representation.
        while not isinstance(representation, Representation):
            representation = representation[np.random.choice(list(representation.keys()))]

        return representation

    def get_random_representations(self, other_genotype):
        if self.__class__.__name__ != other_genotype.__class__.__name__:
            raise Exception("cannot get representations for different genotypes")

        key = np.random.choice(list(self.keys()))

        representation_1 = self[key]
        representation_2 = other_genotype[key]

        # Continue searching in the dictionary chain of keys until we find a key that leads to a representation.
        while not isinstance(representation_1, Representation):
            key = np.random.choice(list(representation_1.keys()))
            representation_1 = representation_1[key]
            representation_2 = representation_2[key]

        return representation_1, representation_2

    @classmethod
    def check(cls, genotype):  # -> Genotype
        if isinstance(genotype, Representation):
            genotype = Genotype(genotype)

        if not isinstance(genotype, Genotype):
            raise Exception("Genotype failure  ", genotype.__class__.__name__)

        return genotype
