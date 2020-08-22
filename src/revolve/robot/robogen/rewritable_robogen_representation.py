from typing import List

from nca.core.abstract.structural.tree.tree_helper import Coordinate3D
from nca.core.genome.representations.symbolic_representation import LSystemRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenModule, RobogenGrammar, RobogenSymbol
from revolve.robot.robogen.robogen_representation import RobogenRepresentation

RobogenAxiom = List[RobogenModule]


class RewritableRobogenRepresentation(RobogenRepresentation, LSystemRepresentation):

    axiom: RobogenAxiom = [RobogenModule(RobogenSymbol.CORE, Coordinate3D(0, 0, 0))]

    def __init__(self, grammar: RobogenGrammar):
        super().__init__(grammar)
