from typing import Dict

from nca.core.genome.grammar.grammar import Axiom
from nca.core.genome.grammar.lsystem_representation import LSystemRepresentation
from revolve.robot.body.robogen.robogen_grammar import RobogenModule, RobogenGrammar

axiom: Axiom = [RobogenModule.MODULE_CORE_COMPONENT]


class RobogenRepresentation(LSystemRepresentation, Tree):

    def __init__(self, grammar: RobogenGrammar):
        super().__init__(grammar)
