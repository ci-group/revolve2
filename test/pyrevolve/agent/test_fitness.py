import unittest

from pyrevolve.evolutionary.robotics import TestFitness


class TestAgentFitness(unittest.TestCase):

    def test_compare(self):
        fitness1 = TestFitness()
        fitness1.fitness = 1

        fitness2 = TestFitness()
        fitness2.fitness = 0

        self.assertTrue(fitness1 > fitness2)
