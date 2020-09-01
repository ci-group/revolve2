from typing import List

import numpy as np

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.grammar import Alphabet


class GrammarInitialization(Initialization):
    def __init__(self, alphabet: type(Alphabet)):
        super().__init__()
        self.alphabet: type(Alphabet) = alphabet

    def __call__(self, size: int) -> List:
        return np.random.choice(self.alphabet, size, p=self.alphabet.probabilities()).tolist()
