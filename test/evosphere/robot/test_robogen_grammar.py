import unittest

from revolve.robot.body.robogen.robogen_module import RobogenModule


class TestRobogenGrammar(unittest.TestCase):

    def test_module(self):
        module_1 = RobogenModule()
        module_2 = RobogenModule()

        self.assertNotEqual(module_1.id, module_2.id)
