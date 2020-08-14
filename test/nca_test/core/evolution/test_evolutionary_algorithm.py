import unittest

from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory
from nca.core.ecology import PopulationEcology
from nca.core.ecology.population_management import PopulationManagement
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from revolve.evosphere.configuration import RevolveGeneticAlgorithmConfiguration


class TestEvolutionaryAlgorithm(unittest.TestCase):

    def test_initialize(self):

        configuration = RevolveGeneticAlgorithmConfiguration()
        evolutionary_algorithm = EvolutionaryAlgorithm(configuration)

        population_ecology = PopulationEcology(PopulationManagement())
        population_ecology.initialize(IndividualFactory().create(10))
        evolutionary_algorithm.initialize(population_ecology.management.populations())

        for population in population_ecology.management.populations():
            for individual in population.individuals:
                self.assertIsNotNone(individual.representation)

    def test_run(self):

        configuration = RevolveGeneticAlgorithmConfiguration()
        evolutionary_algorithm = EvolutionaryAlgorithm(configuration)

        population_ecology = PopulationEcology(PopulationManagement())
        population_ecology.initialize(IndividualFactory().create(10))
        evolutionary_algorithm.initialize(population_ecology.management.populations())

        evolutionary_algorithm.run(population_ecology.management.populations()[0], evaluator)

        for population in population_ecology.management.populations():
            for individual in population.individuals:
                self.assertIsNotNone(individual.representation)


def evaluator(offspring: Agents):
    return offspring
