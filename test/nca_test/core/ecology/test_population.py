import unittest

import numpy as np

from nca.core.actor.actors import Actors
from nca.core.actor.fitness import Fitness
from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology.population import Population


class TestPopulation(unittest.TestCase):
    n = 10

    def test_id(self):
        population1 = Population(ActorFactory().create(self.n))
        population2 = Population(ActorFactory().create(self.n))

        self.assertNotEqual(population1.id, population2.id)

    def test_generation(self):
        agents_start = ActorFactory().create(self.n)
        agents_new = ActorFactory().create(self.n)

        population = Population(agents_start)

        population.next_generation(agents_new)

        self.assertNotEqual(population.individuals, agents_start)
        self.assertEqual(population.individuals, agents_new)

    def test_improvement(self):
        agents1: Actors = ActorFactory().create(self.n)

        agents2: Actors = ActorFactory().create(self.n)
        for agent in agents2:
            agent.fitness = Fitness(1.0)

        population1 = Population(agents1)
        population2 = Population(agents2)

        self.assertTrue(population1.did_improve(agents2))
        self.assertFalse(population2.did_improve(agents1))

    def test_json(self):
        population = Population(ActorFactory().create(self.n))

        for individual in population.individuals:
            individual.fitness.add("test", np.random.randint(0, 100))

        json_data = population.to_json()

        self.assertTrue(json_data['id'] >= 0)
        self.assertTrue(json_data['age'] >= 0)
        self.assertTrue(len(json_data['individuals']) == self.n)
        self.assertTrue(len(json_data['fitness']) == 5)
