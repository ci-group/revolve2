
from enum import Enum
from typing import List


class Symbol(Enum):

    @classmethod
    def probabilities(cls, elements=None):
        if elements is None:
            number_of_elements = cls.__len__()
        else:
            number_of_elements = len(elements)
        uniform_value = 1 / number_of_elements
        return [uniform_value for _ in range(number_of_elements)]


Axiom = List[Symbol]

