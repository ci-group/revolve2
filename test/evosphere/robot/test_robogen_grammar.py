import unittest

from revolve.robot.robogen.robogen_grammar import RobogenSymbol, RobogenModule


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
