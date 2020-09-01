import copy
import unittest

from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.operators.recombination_operator import OnePointCrossover
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class TestRecombinationOperators(unittest.TestCase):

    def test_crossover_recombination(self):
        recombination = OnePointCrossover()
        initialization = UniformInitialization()

        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)

        new_representation = recombination._recombine((Individual(copy.deepcopy(representation_1)), Individual(copy.deepcopy(representation_2))))

        self.assertNotEqual(representation_1, new_representation)
        self.assertNotEqual(representation_2, new_representation)

    def test_multi_crossover(self):
        recombination = OnePointCrossover()
        initialization = UniformInitialization()

        representation_1 = ValuedRepresentation(initialization)
        representation_2 = ValuedRepresentation(initialization)
        representation_3 = ValuedRepresentation(initialization)

        print(representation_1)
        print(representation_2)
        print(representation_3)

        new_representation = recombination(Actors([Individual(representation_1), Individual(representation_2), Individual(representation_3)]))
        self.assertNotEqual(representation_2, new_representation)
        self.assertNotEqual(representation_3, new_representation)

        print(representation_1)
        print(representation_2)
        print(representation_3)
        print(new_representation)
