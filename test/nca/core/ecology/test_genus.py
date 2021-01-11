import math
import unittest

import numpy as np

from nca.core.actor.individual import Individual
from nca.core.actor.individual_factory import ActorFactory, Characterization
from nca.core.ecology.population import Population
from nca.core.ecology.speciation.compatibility import Compatibility
from nca.core.ecology.speciation.genus import Genus
from nca.core.genome.operators.initialization import BinaryInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class GenusTest(unittest.TestCase):
    number_of_species = 3
    number_of_individuals = 10

    def test_add(self):
        genus = Genus(Compatibility(limit=math.inf))

        for _ in range(self.number_of_species):
            genus.add(Population(ActorFactory().create(self.number_of_individuals)))

        self.assertEqual(len(genus.species), self.number_of_species)

    def test_insert(self):
        genus = Genus(Compatibility(limit=math.inf))

        factory = ActorFactory(ValuedRepresentation(BinaryInitialization()))
        genus.add(Population(factory.create(self.number_of_individuals)))

        individual: Individual = factory.create(1)[0]
        genus.insert(individual)

        self.assertEqual(len(genus.species), 1)
        self.assertEqual(len(genus.species[0].individuals), self.number_of_individuals+1)

    def test_json(self):
        genus = Genus(Compatibility(limit=math.inf))

        for _ in range(self.number_of_species):
            population = Population(ActorFactory().create(self.number_of_individuals))
            for individual in population.individuals:
                individual.fitness.append("test", np.random.randint(0, 100))
            genus.add(population)

        json = genus.to_json()
        self.assertTrue(json['id'] >= 0)
        self.assertEqual(len(json['species']), self.number_of_species)
