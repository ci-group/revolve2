import copy
import unittest

from nca.core.agent.fitnesses import DisplacementFitness
from nca.core.agent.individual import Individual
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.representation import Representation
from simulation_test.simulator.mock_measures import MockPerformanceMeasures


class TestIndividual(unittest.TestCase):

    def test_compare(self):

        representation = Representation(UniformInitialization())
        individual = Individual(representation)

        self.assertEqual(individual.representation, representation)
        self.assertEqual(individual.fitness, 0.0)

    def test_id(self):
        representation = Representation(UniformInitialization())
        individual1 = Individual(representation)
        individual2 = Individual(representation)

        self.assertNotEqual(individual1.id, individual2.id)

    def test_performance(self):
        representation = Representation(UniformInitialization())
        individual = Individual(representation)

        new_individual: Individual = copy.deepcopy(individual)
        new_individual.measures = MockPerformanceMeasures()
        new_individual.performance(DisplacementFitness())

        self.assertNotEqual(individual.measures, new_individual.measures)
        self.assertNotEqual(individual.fitness, new_individual.fitness)
