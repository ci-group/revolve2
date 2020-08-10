import random
from enum import Enum
from typing import List, Dict


class Symbol(Enum):

    @classmethod
    def alphabet(cls):
        return list(map(lambda c: c, cls))


Axiom = List[Symbol]

Alphabet = List[Symbol]

ReplacementRules = Dict[Symbol, List[List[Symbol]]]


class SemiThueSystem:

    def __init__(self, alphabet: Alphabet, rules: ReplacementRules):
        self.alphabet = alphabet
        self.rules = rules

    def apply_rules(self, symbols):
        new_symbols = []

        for index, element in enumerate(symbols):
            if element in self.rules.keys():
                new_symbols.extend(random.choice(self.rules[element]))
            else:
                new_symbols.append(element)

        return new_symbols


Grammar = SemiThueSystem
