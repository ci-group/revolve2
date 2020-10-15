import random

from enum import Enum
from typing import List, Dict


class Symbol(Enum):

    def probabilities(cls):
        number_of_elements = cls.__len__()
        uniform_value = 1 / number_of_elements
        return [uniform_value for _ in range(number_of_elements)]


Axiom = List[Symbol]
Alphabet = List[Symbol]
ReplacementRules = Dict[Symbol, List[Alphabet]]


class SemiThueSystem:

    def __init__(self, symbols_type: type(Symbol), rules: ReplacementRules):
        self.symbols_type = symbols_type
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
