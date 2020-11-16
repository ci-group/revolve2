from enum import auto

import numpy as np

from nca.core.genome.grammar.symbol import Symbol


class TestColorSymbol(Symbol):
    RED = auto()
    GREEN = auto()
    BLUE = auto()

    @classmethod
    def probabilities(cls):
        return np.array([0.65, 0.25, 0.10])
