from nca.core.genome.grammar.grammar import Axiom
from nca.core.genome.grammar.lsystem_representation import LSystemRepresentation
from nca.core.genome.representations.tree import Tree
from revolve.robot.body.robogen.robogen_grammar import RobogenModule, RobogenGrammar

axiom: Axiom = [RobogenModule.CORE]


class RobogenRepresentation(LSystemRepresentation):

    def __init__(self, grammar: RobogenGrammar):
        super().__init__(grammar)
