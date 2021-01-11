import unittest

from revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype
from revolve.robot.body.robogen.robogen_word import RobogenWord
from revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robot_visualizer import generate_matrix


class RobogenRepresentationTest(unittest.TestCase):

    def test_rewritable_grammar(self):
        #for i in range(10):
        representation = IndirectRobogenGenotype()
        representation(rule_iterations=2)

        body = RobogenBodyBuilder()
        body_modules = body.develop(representation.encoding)

        matrix = generate_matrix(body_modules)
        #show(matrix)

    def test_rewritable_grammar(self):
        #for i in range(10):
        representation = IndirectRobogenGenotype(RobogenWord.generate_rules(), RobogenWord.generate_axiom())
        representation(rule_iterations=2)

        body = RobogenBodyBuilder()
        body_modules = body.develop(representation.encoding)

        matrix = generate_matrix(body_modules)
        #show(matrix)

    def test_replace_joint(self):
        axiom = []
        block_word = RobogenWord(RobogenSymbol.MODULE_BLOCK).symbols()
        joint1_word = RobogenWord(RobogenSymbol.MODULE_HORIZONTAL_JOINT).symbols()
        axiom.extend(joint1_word)
        axiom.extend(block_word)
        axiom.extend(joint1_word)
        axiom.extend(block_word)

        replacement_rules = {joint1_word[0]: [RobogenWord(RobogenSymbol.MODULE_BLOCK,
                                                                RobogenSymbol.ORIENTATION_TOP).symbols()]}
        representation = IndirectRobogenGenotype(rules=replacement_rules, axiom=axiom)
        representation(rule_iterations=1)

        body = RobogenBodyBuilder()
        body_modules = body.develop(representation.encoding)

        matrix = generate_matrix(body_modules)
        #show(matrix)

    def test_snake(self):
        axiom = []
        block_word = RobogenWord(RobogenSymbol.MODULE_BLOCK).symbols()
        joint1_word = RobogenWord(RobogenSymbol.MODULE_HORIZONTAL_JOINT).symbols()
        joint2_word = RobogenWord(RobogenSymbol.MODULE_VERTICAL_JOINT).symbols()
        axiom.extend(joint1_word)
        axiom.extend(block_word)
        axiom.extend(joint2_word)
        axiom.extend(block_word)
        axiom.extend(joint1_word)
        axiom.extend(block_word)
        axiom.extend(joint2_word)
        axiom.extend(block_word)

        algorithm = IndirectRobogenGenotype(axiom=axiom)

        body = RobogenBodyBuilder()
        body_modules = body.develop(algorithm.representation)

        matrix = generate_matrix(body_modules)
        #show(matrix)

    def test_spider(self):
        axiom = []

        def add_limb(orientation: RobogenSymbol = RobogenSymbol.ORIENTATION_TOP):
            axiom.append(RobogenSymbol.BRACKET_STASH)
            block_word = RobogenWord(RobogenSymbol.MODULE_BLOCK, orientation).symbols()
            joint1_word = RobogenWord(RobogenSymbol.MODULE_HORIZONTAL_JOINT, orientation).symbols()
            joint2_word = RobogenWord(RobogenSymbol.MODULE_VERTICAL_JOINT, orientation).symbols()
            axiom.extend(joint1_word)
            axiom.extend(block_word)
            axiom.extend(joint2_word)
            axiom.extend(block_word)
            axiom.append(RobogenSymbol.BRACKET_POP)

        add_limb(RobogenSymbol.ORIENTATION_TOP)
        add_limb(RobogenSymbol.ORIENTATION_LEFT)
        add_limb(RobogenSymbol.ORIENTATION_RIGHT)
        add_limb(RobogenSymbol.ORIENTATION_DOWN)

        algorithm = IndirectRobogenGenotype(axiom=axiom)

        body = RobogenBodyBuilder()
        body_modules = body.develop(algorithm.representation)

        matrix = generate_matrix(body_modules)
        #show(matrix)

    def test_gecko(self):
        axiom = []

        def add_limb(orientation: RobogenSymbol = RobogenSymbol.ORIENTATION_TOP):
            axiom.append(RobogenSymbol.BRACKET_STASH)
            block_word = RobogenWord(RobogenSymbol.MODULE_BLOCK, orientation).symbols()
            joint1_word = RobogenWord(RobogenSymbol.MODULE_HORIZONTAL_JOINT, orientation).symbols()
            axiom.extend(joint1_word)
            axiom.extend(block_word)
            axiom.append(RobogenSymbol.BRACKET_POP)

        add_limb(RobogenSymbol.ORIENTATION_LEFT)
        add_limb(RobogenSymbol.ORIENTATION_RIGHT)
        axiom.append(RobogenSymbol.BRACKET_STASH)

        block_word = RobogenWord(RobogenSymbol.MODULE_BLOCK, RobogenSymbol.ORIENTATION_DOWN).symbols()
        joint2_word = RobogenWord(RobogenSymbol.MODULE_VERTICAL_JOINT, RobogenSymbol.ORIENTATION_DOWN).symbols()
        axiom.extend(joint2_word)
        axiom.extend(block_word)
        axiom.extend(joint2_word)
        axiom.extend(block_word)

        add_limb(RobogenSymbol.ORIENTATION_LEFT)
        add_limb(RobogenSymbol.ORIENTATION_RIGHT)

        axiom.append(RobogenSymbol.BRACKET_POP)

        algorithm = IndirectRobogenGenotype(axiom=axiom)

        body = RobogenBodyBuilder()
        body_modules = body.develop(algorithm.representation)

        matrix = generate_matrix(body_modules)
        #show(matrix)
