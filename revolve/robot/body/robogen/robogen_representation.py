from typing import Dict

from nca.core.genome.grammar import LSystemRepresentation
from revolve.robot.morphology import RobogenAlphabet, RobogenModuleAlphabet

axiom = RobogenModuleAlphabet.MODULE_CORE_COMPONENT

RobogenRules = Dict[RobogenAlphabet, RobogenAlphabet]


class RobogenRepresentation(LSystemRepresentation):

    def __init__(self, alphabet: type(RobogenAlphabet) = RobogenAlphabet, rules: RobogenRules = RobogenRules):
        super().__init__(alphabet=alphabet, rules=rules)
