import copy
import unittest

from nca.core.actor.fitnesses import DisplacementFitness
from nca.core.actor.individual import Individual
from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.robot.robot import Robot
from simulation_test.simulator.mock_measures import MockPerformanceMeasures


class TestRobogenManipulation(unittest.TestCase):

    def test_performance(self):
        representation = ValuedRepresentation(UniformInitialization())
        robot = Robot(representation)
        fitness = DisplacementFitness()

        new_robot: Robot = copy.deepcopy(robot)
        new_robot.measures = MockPerformanceMeasures()
        fitness(new_robot)

        self.assertNotEqual(new_robot.measures, robot.measures)
        self.assertNotEqual(new_robot.fitness, robot.fitness)
