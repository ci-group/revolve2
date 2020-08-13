import unittest

from nca.core.agent.fitness import Fitness
from nca.core.genome.representation import Representation
from revolve.robot.birth_clinic import BirthClinic
from revolve.robot.body.body import Body
from revolve.robot.body.body_builder import BodyBuilder
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol, RobogenModule
from revolve.robot.brain.brain import Brain
from src.revolve.robot.brain.brain_builder import BrainBuilder


class TestRobogenGrammar(unittest.TestCase):

    def test_symbols(self):
        symbol = RobogenSymbol

        self.assertEqual(symbol.probabilities()[RobogenSymbol.CORE.value - 1], 0.0)
        self.assertEqual(symbol.probabilities()[RobogenSymbol.SENSOR.value - 1], 0.0)

        self.assertNotEqual(symbol.probabilities()[RobogenSymbol.HORIZONTAL_JOINT.value - 1], 0.0)
        self.assertNotEqual(symbol.probabilities()[RobogenSymbol.VERTICAL_JOINT.value - 1], 0.0)
        self.assertNotEqual(symbol.probabilities()[RobogenSymbol.BLOCK.value - 1], 0.0)

    def test_module(self):
        module_1 = RobogenModule()
        module_2 = RobogenModule()

        self.assertNotEqual(module_1.id, module_2.id)
