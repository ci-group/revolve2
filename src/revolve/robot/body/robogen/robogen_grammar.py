from enum import auto
from typing import Dict, List

from nca.core.genome.grammar.grammar import Symbol, Grammar
from nca.core.genome.representations.tree import Tree, Tree2D


class RobogenSymbol(Symbol):
    pass


class RobogenModule(RobogenSymbol):
    CORE = auto()
    HORIZONTAL_JOINT = auto()
    VERTICAL_JOINT = auto()
    BLOCK = auto()
    SENSOR = auto()

    @classmethod
    def probabilities(self):
        return [0.0, 0.25, 0.25, 0.5, 0.0]


class RobogenMounting(RobogenSymbol):
    RIGHT = auto()
    FRONT = auto()
    LEFT = auto()


def RobogenSymbols():
    symbols = []
    symbols.extend(RobogenModule.alphabet())
    symbols.extend(RobogenMounting.alphabet())
    return symbols


RobogenAlphabet = List[RobogenSymbol]
RobogenOperators: Dict[str, RobogenAlphabet] = {'module': RobogenModule.alphabet(),
                                                'mounting': RobogenMounting.alphabet()}
RobogenRules = Dict[RobogenSymbol, List[RobogenAlphabet]]


class RobogenGrammar(Grammar, Tree):

    def __init__(self, alphabet: RobogenAlphabet, rules: RobogenRules):
        super().__init__(alphabet, rules)

