from typing import List

import numpy as np

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.grammar import Symbol


class GrammarInitialization(Initialization):
    def __init__(self, symbol_type: type(Symbol)):
        super().__init__()
        self.symbol_type: type(Symbol) = symbol_type

    def __call__(self, size: int) -> List:
        return np.random.choice(self.symbol_type, size, p=self.symbol_type.probabilities()).tolist()
