from enum import Enum
import numpy as np


class Alphabet(Enum):

    @classmethod
    def list(cls):
        return list(map(lambda c: c, cls))

    @classmethod
    def probabilities(cls):
        number_of_elements = len(cls.list())
        return np.ones([number_of_elements]) * 1 / number_of_elements
