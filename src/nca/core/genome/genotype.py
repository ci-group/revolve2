from typing import Dict, List

from nca.core.genome.representation import Representation


class Genotype(Dict[str, Representation]):

    def __init__(self, representations, keys=None):
        super().__init__()

        if isinstance(representations, Representation):
            self._initialize_single_representation(representations, keys)
        else:
            self._initialize_multiple_representations(representations, keys)

    def _initialize_single_representation(self, representation: Representation, key=None):
        if key is None:
            key = str(type(representation).__name__)
        self[key] = representation

    def _initialize_multiple_representations(self, representations: List[Representation], keys: List[str] = None):
        for index, representation in enumerate(representations):
            if keys is None:
                key = representation.__class__.__name__
                if key in self.keys():
                    raise Exception("Genotype: Cannot add multiple representations of the same type without specifying keys.")
            else:
                key = keys[index]
            self[key] = representation

    def compatibility(self, other):
        sum = 0
        if len(self.keys()) > 1:
            key = list(self.keys())[0]
            sum += self[key].compatibility(other[key])
        else:
            for key in self.keys():
                sum += self[key].compatibility(other[key])

        return sum