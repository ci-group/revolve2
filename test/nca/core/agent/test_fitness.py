import unittest

from revolve2.nca.core.actor.fitness_evaluation import OnesFitness
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.genome.operators.initialization import UniformInitialization, BinaryInitialization
from revolve2.nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve2.revolve.evosphere.fitness_evaluation import DisplacementFitnessEvaluation, RotationalFitnessEvaluation, DisplacementRotationalFitnessEvaluations
from revolve2.revolve.robot.robot import Robot
from test.simulation.simulator.mock_measures import MockPerformanceMeasures


class TestFitness(unittest.TestCase):

    def test_displacement(self):

        fitness = DisplacementFitnessEvaluation()
        initialization = BinaryInitialization()
        individual = Robot(ValuedRepresentation(initialization))
        individual.measures = MockPerformanceMeasures()

        fitness_value = fitness(individual)
        self.assertNotEqual(fitness_value, 0.0)

    def test_rotational(self):

        fitness = RotationalFitnessEvaluation()
        individual = Robot(ValuedRepresentation(initialization=BinaryInitialization()))
        individual.measures = MockPerformanceMeasures()

        fitness_value = fitness(individual)
        self.assertNotEqual(fitness_value, 0.0)

    def test_ones(self):

        fitness = OnesFitness()
        individual = Individual(ValuedRepresentation(initialization=UniformInitialization()))

        fitness_value = fitness(individual)
        self.assertNotEqual(fitness_value, 0.0)

    def test_multi(self):

        fitness_evaluation = DisplacementRotationalFitnessEvaluations()
        individual = Individual(ValuedRepresentation(initialization=BinaryInitialization()))
        individual.measures = MockPerformanceMeasures()
        fitness_value = fitness_evaluation(individual)
        self.assertNotEqual(fitness_value, 0.0)
