import unittest
from typing import List

from revolve2.nca.comparative_evolution import ComparativeEvolution
from revolve2.nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve2.nca.core.genome.operators.recombination_operator import OnePointCrossover, UniformCrossover, \
    OnePointUniformCrossover, OneElementCrossover, AllElementCrossover


class TestComparitiveEvolution(unittest.TestCase):

    def test_create(self):
        configurations: List[EvolutionaryConfiguration] = \
        [
            GeneticAlgorithmConfiguration(recombination=OnePointCrossover()),
            GeneticAlgorithmConfiguration(recombination=OnePointUniformCrossover()),
            GeneticAlgorithmConfiguration(recombination=OneElementCrossover()),
            GeneticAlgorithmConfiguration(recombination=AllElementCrossover()),
            GeneticAlgorithmConfiguration(recombination=UniformCrossover())
        ]
        evolution = ComparativeEvolution(configurations,
                                         algorithm_names=["OnePointCrossover", "OnePointUniformCrossover",
                                                          "OneElementCrossover", "AllElementCrossover",
                                                          "UniformCrossover"], repeat=1)

        evolution.gather()

        self.assertTrue(True)
