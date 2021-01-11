import unittest

from nca.core.actor.fitness import Fitness
from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology.population import Population
from nca.core.evolution.conditions.condition import FitnessCondition, EvaluationsCondition, \
    ImprovementCondition


class TestTerminationConditions(unittest.TestCase):
    n = 3

    def test_fitness_condition_stop(self):
        population = Population(ActorFactory().create(self.n))
        termination_fitness = 0.5
        population.individuals[1].fitness = Fitness(termination_fitness)

        termination_condition = FitnessCondition(termination_fitness)

        self.assertTrue(termination_condition.terminate(population))

    def test_fitness_condition_continue(self):
        population = Population(ActorFactory().create(self.n))
        population.individuals[1].fitness = Fitness(0.4)

        termination_condition = FitnessCondition(0.5)

        self.assertFalse(termination_condition.terminate(population))

    def test_generations_condition_stop(self):
        population = Population(ActorFactory().create(self.n))
        generations_condition = 10
        population.age.generations = generations_condition

        termination_condition = EvaluationsCondition(generations_condition)

        self.assertTrue(termination_condition.terminate(population))

    def test_generations_condition_continue(self):
        population = Population(ActorFactory().create(self.n))
        population.age.generations = 9

        termination_condition = EvaluationsCondition(10)

        self.assertFalse(termination_condition.terminate(population))

    def test_improvements_condition_stop(self):
        population = Population(ActorFactory().create(self.n))

        no_improvements_count = 5

        population.age.no_improvement_count = no_improvements_count

        termination_condition = ImprovementCondition(no_improvements_count)

        self.assertTrue(termination_condition.terminate(population))

    def test_improvements_condition_continue(self):
        population = Population(ActorFactory().create(self.n))

        population.age.no_improvement_count = 4

        termination_condition = ImprovementCondition(5)

        self.assertFalse(termination_condition.terminate(population))

