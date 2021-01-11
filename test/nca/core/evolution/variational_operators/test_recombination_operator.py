import copy
import unittest

from nca.core.actor.individual import Individual
from nca.core.genome.genotype import Genotype
from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class TestRecombinationOperators(unittest.TestCase):

    def test_recombination_algorithm(self):
        recombination = OnePointCrossover()

        genotype_1 = Genotype(ValuedRepresentation())
        genotype_2 = Genotype(ValuedRepresentation())
        parents = [Individual(genotype_1), Individual(genotype_2)]
        new_genotype = recombination(parents, debug=True)
        self.assertNotEqual(genotype_1, new_genotype)
        self.assertNotEqual(genotype_2, new_genotype)

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()

        representation_1 = ValuedRepresentation()
        representation_2 = ValuedRepresentation()

        new_representation = recombination._recombine(copy.deepcopy(representation_1), copy.deepcopy(representation_2))

        self.assertNotEqual(representation_1, new_representation)
        self.assertNotEqual(representation_2, new_representation)

    def test_multi_crossover(self):
        recombination = OnePointCrossover()
        initialization = UniformInitialization()

        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)
        representation_3 = ValuedRepresentation(initialization)
        genotypes = [individual for individual in [Individual(representation_1), Individual(representation_2), Individual(representation_3)]]
        new_genotype = recombination(genotypes, debug=True)
        self.assertNotEqual(representation_1, new_genotype['ValuedRepresentation'])
        self.assertNotEqual(representation_2, new_genotype['ValuedRepresentation'])
        self.assertNotEqual(representation_3, new_genotype['ValuedRepresentation'])
