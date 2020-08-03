from typing import Dict

from nca.core.genome.representations.l_system.alphabet import Alphabet
from nca.core.genome.representations.direct_representation import GrammarRepresentation


Rules = Dict[Alphabet, Alphabet]


class LSystemRepresentation(GrammarRepresentation):

    def __init__(self, alphabet: type(Alphabet), rules: Rules):
        super().__init__(alphabet=alphabet)
        self.rules: Rules = rules

    def apply_rules(self):
        for index, element in enumerate(self.genome):
            if element in self.rules.keys():
                self.genome.remove(element)
                self.genome.insert(index, self.rules[element])
