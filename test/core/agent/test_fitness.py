import random
import unittest

from nca.core.agent.fitness import Fitness
from simulation.measures import Measures


class TestFitness(Fitness):

    def __init__(self):
        super().__init__()

    def calculate(self, measures: Measures):
        self.fitness = random.random()


class TestAgentFitness(unittest.TestCase):

    def test_compare(self):
        fitness1 = TestFitness()
        fitness1.fitness = 1

        fitness2 = TestFitness()
        fitness2.fitness = 0

        self.assertTrue(fitness1 > fitness2)
