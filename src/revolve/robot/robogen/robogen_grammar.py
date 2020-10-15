import random
from enum import auto
from typing import List, Dict

import numpy as np

from nca.core.abstract.structural.tree.tree_helper import Coordinate3D, Orientation
from nca.core.abstract.sequential_identifier import NodeIdentifier
from nca.core.genome.grammar.grammar import Symbol, Grammar, Alphabet


class RobogenSymbol(Symbol):

    MODULE_CORE = auto()
    MODULE_BLOCK = auto()
    MODULE_HORIZONTAL_JOINT = auto()
    MODULE_VERTICAL_JOINT = auto()
    MODULE_SENSOR = auto()

    ORIENTATION_TOP = Orientation.TOP
    ORIENTATION_RIGHT = Orientation.RIGHT
    ORIENTATION_LEFT = Orientation.LEFT
    ORIENTATION_DOWN = Orientation.DOWN

    BRACKET_STASH = auto()
    BRACKET_POP = auto()

    @classmethod
    def random_words(cls, size: int = 1):
        sequence = []
        for i in range(size):
            orientation = np.random.choice(RobogenSymbol.orientation(), p=[0.25, 0.25, 0.25, 0.25])
            module = np.random.choice(RobogenSymbol.modules(), p=[0.50, 0.25, 0.25])
            sequence.append({'orientation': orientation, 'module': module})
        return sequence

    @classmethod
    def orientation(cls):
        return [RobogenSymbol.ORIENTATION_TOP, RobogenSymbol.ORIENTATION_RIGHT, RobogenSymbol.ORIENTATION_LEFT, RobogenSymbol.ORIENTATION_DOWN]

    @classmethod
    def modules(cls):
        return [RobogenSymbol.MODULE_BLOCK, RobogenSymbol.MODULE_HORIZONTAL_JOINT, RobogenSymbol.MODULE_VERTICAL_JOINT]

    @classmethod
    def probabilities(cls):
        probabilities = np.array([0.0, 0.50, 0.25, 0.25, 0.0, 0.25, 0.25, 0.25, 0.25, 0.0, 0.0])
        return probabilities / sum(probabilities)


class RobogenModule:

    identifier = NodeIdentifier()
    symbol_type = RobogenSymbol

    def __init__(self, symbol: RobogenSymbol = RobogenSymbol.MODULE_CORE, coordinate: Coordinate3D = Coordinate3D(0, 0, 0)):
        self.id = self.identifier.id()
        self.symbol = symbol
        self.coordinate: Coordinate3D = coordinate

    def __repr__(self):
        return "(" + self.symbol.name + ", " + str(self.id) + ", " + str(self.coordinate) + ")"


RobogenReplacementRules = Dict[Symbol, List[List[Alphabet]]]


class RobogenGrammar(Grammar):

    def __init__(self, rules: RobogenReplacementRules):
        super().__init__(RobogenSymbol, rules)

    def apply_rules(self, words):
        new_words = []
        for index, word in enumerate(words):
            module = word['module']
            if module in self.rules.keys():
                new_words.extend(random.choice(self.rules[module]))
            else:
                new_words.append(word)
        return new_words
