from enum import auto
from typing import Dict, List

from nca.core.abstract.composite.tree_helper import Coordinate3D
from nca.core.abstract.sequential_identifier import NodeIdentifier
from nca.core.genome.grammar.grammar import Symbol, Grammar


class RobogenSymbol(Symbol):
    CORE = auto()
    HORIZONTAL_JOINT = auto()
    VERTICAL_JOINT = auto()
    BLOCK = auto()
    SENSOR = auto()

    @classmethod
    def probabilities(cls):
        return [0.0, 0.25, 0.25, 0.5, 0.0]


class RobogenModule:

    identifier = NodeIdentifier()
    symbol_type = RobogenSymbol

    def __init__(self, symbol: RobogenSymbol = RobogenSymbol.CORE, coordinate: Coordinate3D = Coordinate3D(0, 0, 0)):
        self.id = self.identifier.id()
        self.symbol = symbol
        self.coordinate: Coordinate3D = coordinate

    def __repr__(self):
        return "(" + self.symbol.name + ", " + str(self.id) + ", " + str(self.coordinate) + ")"


RobogenAlphabet = List[RobogenSymbol]
RobogenRules = Dict[RobogenSymbol, List[RobogenAlphabet]]


class RobogenGrammar(Grammar):

    def __init__(self, rules: RobogenRules):
        super().__init__(RobogenSymbol, rules)
