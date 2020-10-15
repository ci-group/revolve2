import unittest

from nca.core.genome.grammar.grammar import Grammar, ReplacementRules
from revolve.robot.body.robogen_body import RobogenBody
from revolve.robot.robogen.rewritable_robogen_representation import RewritableRobogenRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenSymbol, RobogenGrammar, RobogenReplacementRules
from revolve.robot.robogen.robot_visualizer import generate_matrix, show


class LSystemRepresentationTest(unittest.TestCase):

    def test_rewritable(self):
        representation = RewritableRobogenRepresentation()
        for i in range(2):
            representation(rule_iterations=1)

        body = RobogenBody()

        body_modules = body.develop(representation)
        print(body_modules)
        matrix = generate_matrix(body_modules)
        show(matrix)
