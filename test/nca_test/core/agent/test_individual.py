import copy
import math
import unittest

from nca.core.agent.fitness import DisplacementFitness
from nca.core.agent.individual import Individual
from nca.core.genome.representation import Representation
from nca_test.core.agent.test_measures import MockSimulationMeasures


class TestIndividual(unittest.TestCase):

    def test_compare(self):

        representation = Representation()
        individual = Individual(representation)

        self.assertEqual(individual.representation, representation)
        self.assertEqual(individual.fitness, -math.inf)

    def test_id(self):
        representation = Representation()
        individual1 = Individual(representation)
        individual2 = Individual(representation)

        self.assertNotEqual(individual1.id, individual2.id)

    def test_performance(self):
        representation = Representation()
        individual = Individual(representation)
        measures = MockSimulationMeasures()
        fitness = DisplacementFitness()

        new_individual: Individual = copy.deepcopy(individual)
        new_individual.performance(measures, fitness)

        self.assertNotEqual(individual.measures, new_individual.measures)
        self.assertNotEqual(individual.fitness, new_individual.fitness)

