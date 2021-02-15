import copy
import unittest

from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.evosphere.fitness_evaluation import DisplacementFitnessEvaluation
from revolve.robot.robot import Robot
from test.simulation.simulator.mock_measures import MockPerformanceMeasures


class TestRobogenManipulation(unittest.TestCase):

    def test_performance(self):
        representation = ValuedRepresentation(UniformInitialization())
        robot = Robot(representation)
        fitness = DisplacementFitnessEvaluation()

        new_robot: Robot = copy.deepcopy(robot)
        new_robot.measures = MockPerformanceMeasures()
        new_robot.fitness.add({'displacement': fitness(new_robot)})

        self.assertNotEqual(new_robot.measures, robot.measures)
        self.assertNotEqual(new_robot.fitness.value(), robot.fitness.value())
