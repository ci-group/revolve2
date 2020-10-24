import copy
import unittest

from nca.core.actor.fitnesses import DisplacementFitness, OnesFitness
from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from nca.core.genome.initialization import UniformInitialization, IntegerInitialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.robot.robot import Robot
from simulation_test.simulator.mock_measures import MockPerformanceMeasures


class TestIndividual(unittest.TestCase):

    def test_compare(self):

        representation = ValuedRepresentation(UniformInitialization())
        individual = Individual(representation)

        self.assertEqual(individual.get_representation(), representation)
        self.assertEqual(individual.get_fitness(), 0.0)

    def test_genotype(self):

        genotype = Genotype([ValuedRepresentation(), ValuedRepresentation()], keys=["values1", "values2"])
        individual = Individual(genotype)

        self.assertEqual(individual.get_representation(), genotype)
        self.assertEqual(individual.get_fitness(), 0.0)


    def test_id(self):
        representation = ValuedRepresentation(UniformInitialization())
        individual1 = Individual(representation)
        individual2 = Individual(representation)

        self.assertNotEqual(individual1.id, individual2.id)

    def test_performance(self):
        representation = ValuedRepresentation(IntegerInitialization())
        individual: Individual = Individual(representation)

        new_individual: Individual = copy.deepcopy(individual)
        fitness = OnesFitness()(new_individual)

        self.assertNotEqual(fitness, 0.0)
        self.assertNotEqual(individual.get_fitness(), new_individual.get_fitness())
