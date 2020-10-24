from typing import List

from nca.core.actor.fitness import Fitness
from nca.core.actor.fitnesses import OnesFitness
from nca.core.actor.individual_factory import ActorFactory
from nca.core.analysis.statistics import Statistics
from nca.core.analysis.summary import Summary
from nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from nca.evolution import Evolution


class RepeatedEvolution(Evolution):

    def __init__(self, evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 fitness: Fitness = OnesFitness(), individual_factory: ActorFactory = ActorFactory(),
                 debug=True, repetitions: int = 10):
        super().__init__(evolutionary_configuration, fitness, individual_factory, debug)
        self.repetitions: int = repetitions

    def evolve(self):
        statistics_list: List[Statistics] = []
        for _ in range(self.repetitions):
            statistics_list.append(super().evolve())

        return Summary().analyze(statistics_list)
