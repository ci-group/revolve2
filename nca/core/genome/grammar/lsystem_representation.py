from typing import Dict, List

from nca.core.genome.representation import Genome
from nca.core.genome.grammar.alphabet import Alphabet
from nca.core.genome.representations.direct_representation import GrammarRepresentation


Rules = Dict[Alphabet, List[Alphabet]]


class LSystemRepresentation(GrammarRepresentation):

    def __init__(self, alphabet: type(Alphabet), rules: Rules):
        super().__init__(alphabet=alphabet)
        self.rules: Rules = rules

    def apply_rules(self):
        new_genome: Genome = []

        for index, element in enumerate(self.genome):
            if element in self.rules.keys():
                new_genome.extend(self.rules[element])
            else:
                new_genome.append(element)

        self.genome = new_genome
