from typing import List

from revolve2.nca.core.actor.fitness_evaluation import OnesFitness, FitnessEvaluation
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology import PopulationEcology
from revolve2.nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve2.nca.evolution import Evolution


class RepeatedEvolution(Evolution):

    def __init__(self, evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 fitness_evaluation: FitnessEvaluation = OnesFitness(), individual_factory: ActorFactory = ActorFactory(),
                 debug=True, repetitions: int = 10):
        super().__init__(evolutionary_configuration, fitness_evaluation, individual_factory, debug)
        self.repetitions: int = repetitions

    def evolve(self):
        results: List[PopulationEcology] = []
        for _ in range(self.repetitions):
            results.append(super().evolve())

        #return Summary().analyze(statistics_list)
