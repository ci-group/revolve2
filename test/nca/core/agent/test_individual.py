import copy
import unittest
from typing import List

from nca.core.actor.fitness_evaluation import OnesFitness
from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from nca.core.genome.operators.initialization import UniformInitialization, IntegerInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation


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

    def test_parents(self):
        individual_1 = Individual(Genotype(ValuedRepresentation()))
        individual_2 = Individual(Genotype(ValuedRepresentation()))
        genotype_3 = Genotype(ValuedRepresentation())
        parents: List[Individual] = [individual_1, individual_2]

        new_individual = Individual(genotype_3, parents)

        self.assertEqual(new_individual.parent_ids[0], parents[0].id)
        self.assertEqual(new_individual.parent_ids[1], parents[1].id)
