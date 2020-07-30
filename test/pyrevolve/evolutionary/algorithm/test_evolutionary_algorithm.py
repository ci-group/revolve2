import unittest

from pyrevolve.evolutionary.algorithm.evolutionary_algorithm import EvolutionaryAlgorithm
from pyrevolve.evolutionary.algorithm.evolutionary_configurations import EvolutionConfiguration, \
    GeneticAlgorithmConfiguration
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import BinaryRepresentation
from pyrevolve.evolutionary.ecology import PopulationEcology
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.individual_factory import IndividualFactory


class TestTerminationConditions(unittest.TestCase):

    def test_initialize(self):

        population_ecology = PopulationEcology(PopulationManagement())
        population_ecology.initialize(IndividualFactory(BinaryRepresentation).create(10))

        configuration = GeneticAlgorithmConfiguration()

        evolutionary_algorithm = EvolutionaryAlgorithm(configuration)

        evolutionary_algorithm.initialize(population_ecology.populations())
