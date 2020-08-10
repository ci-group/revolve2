import numpy as np

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.grammar.grammar import Alphabet
from nca.core.genome.representation import Genome


class GrammarInitialization(Initialization):
    def __init__(self, alphabet: type(Alphabet)):
        super().__init__()
        self.alphabet: type(Alphabet) = alphabet

    def algorithm(self, size: int) -> Genome:
        return np.random.choice(self.alphabet, size, p=self.alphabet.probabilities())
