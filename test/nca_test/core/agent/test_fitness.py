import unittest

from nca.core.actor.fitnesses import DisplacementFitness, RotationalFitness, OnesFitness, DisplacementRotationalFitness
from nca.core.actor.individual import Individual
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import UniformInitialization, BinaryInitialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from simulation_test.simulator.mock_measures import MockPerformanceMeasures


class TestFitness(unittest.TestCase):

    def test_displacement(self):

        fitness = DisplacementFitness()
        initialization = BinaryInitialization()
        individual = Individual(ValuedRepresentation(initialization))
        individual.measures = MockPerformanceMeasures()

        self.assertEqual(individual.fitness, 0.0)
        individual.fitness = fitness(individual)
        self.assertNotEqual(individual.fitness, 0.0)

    def test_rotational(self):

        fitness = RotationalFitness()
        individual = Individual(ValuedRepresentation(initialization=BinaryInitialization()))
        individual.measures = MockPerformanceMeasures()

        self.assertEqual(individual.fitness, 0.0)
        individual.fitness = fitness(individual)
        self.assertNotEqual(individual.fitness, 0.0)

    def test_ones(self):

        fitness = OnesFitness()
        individual = Individual(ValuedRepresentation(initialization=UniformInitialization()))

        self.assertEqual(fitness, 0.0)
        fitness = fitness(individual)
        self.assertNotEqual(fitness, 0.0)

    def test_multi(self):

        fitness = DisplacementRotationalFitness()
        individual = Individual(ValuedRepresentation(initialization=BinaryInitialization()))
        individual.measures = MockPerformanceMeasures()

        self.assertEqual(fitness, 0.0)
        self.assertEqual(fitness.fitnesses, [0.0, 0.0])
        fitness = fitness(individual)
        self.assertNotEqual(fitness.fitnesses, [])
        self.assertNotEqual(fitness, 0.0)

    def test_hybrid(self):

        individual = Individual(ValuedRepresentation(initialization=BinaryInitialization()))
        individual.measures = MockPerformanceMeasures()

        self.assertEqual(individual.fitness, 0.0)
        self.assertEqual(individual.fitness.fitnesses, [])
        individual.performance(DisplacementFitness())
        self.assertNotEqual(individual.fitness.fitnesses, [])
        self.assertNotEqual(individual.fitness, 0.0)

    def test_hybrid_partial(self):

        individual = Individual(ValuedRepresentation(initialization=BinaryInitialization()))
        individual.measures = MockPerformanceMeasures()

        self.assertEqual(individual.fitness, 0.0)
        self.assertEqual(individual.fitness.fitnesses, [])
        individual.performance(DisplacementFitness())
        self.assertNotEqual(individual.fitness.fitnesses, [])
        self.assertNotEqual(individual.fitness, 0.0)
