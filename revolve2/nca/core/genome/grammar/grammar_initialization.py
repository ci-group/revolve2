from typing import List

import numpy as np

from revolve2.nca.core.evolution.conditions.initialization import Initialization
from revolve2.nca.core.genome.grammar.symbol import Symbol
from revolve2.revolve.robot.body.robogen.robogen_module import RobogenModule

RobogenAxiom = List[RobogenModule]


class GrammarInitialization(Initialization):
    def __init__(self, symbol_type: type(Symbol)):
        super().__init__()
        self.symbol_type: type(Symbol) = symbol_type

    def __call__(self, size: int) -> List:
        return np.random.choice(self.symbol_type, size, p=self.symbol_type.probabilities()).tolist()


class LSystemInitialization(GrammarInitialization):
    def __init__(self, symbol_type: type(Symbol), axiom_initialization=None, rules_initialization=None):
        super().__init__(symbol_type)
        self.axiom_initialization = axiom_initialization
        self.rules_initialization = rules_initialization
