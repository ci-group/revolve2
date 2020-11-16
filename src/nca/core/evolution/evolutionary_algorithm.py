import time
from typing import List

from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.ecology.population import Population
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.conditions.special_features import SpecialFeatures
from nca.core.evolution.conditions.condition import Condition
from nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from nca.core.evolution.selection.mortality_selection import NullMortalitySelection
from nca.core.evolution.selection.selection import ParentSelection, SurvivorSelection, MortalitySelection
from nca.core.genome.operators.mutation_operator import MutationOperator
from nca.core.genome.operators.recombination_operator import RecombinationOperator


class EvolutionaryAlgorithm:

    def __init__(self, configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration()):
        self.configuration: EvolutionaryConfiguration = configuration

        self.parent_selection: ParentSelection = self.configuration.parent_selection
        self.survivor_selection: SurvivorSelection = self.configuration.survivor_selection
        self.mortality_selection: MortalitySelection = NullMortalitySelection()

        self.recombination: RecombinationOperator = self.configuration.recombination
        self.mutation: MutationOperator = self.configuration.mutation

        self.initialization_type: type(Initialization) = self.configuration.initialization_type
        self.termination_condition: Condition = self.configuration.condition
        self.special_features: SpecialFeatures = self.configuration.special_features

        self.duration = []

    def should_terminate(self, population: Population):
        return self.termination_condition.terminate(population)

    def run(self, population: Population, evaluator):
        # (PARENT) SELECT parents

        start_time = time.time()
        parent_combinations: List[List[Individual]] = self.parent_selection.select(population.individuals)

        # RECOMBINE, then MUTATE resulting recombination to create resulting offspring
        offspring: Actors = Actors([Individual(self.mutation(self.recombination(parents)), parents)
                                    for parents in parent_combinations])

        # EVALUATE the candidates
        evaluator(offspring)

        population.individuals = self.mortality_selection.algorithm(population.individuals, offspring)

        # (SURVIVOR) SELECT individuals for the next generation
        population.next_generation(self.survivor_selection.select(population.individuals))
        evolutionary_algorithm_time = time.time() - start_time

        self.duration.append(evolutionary_algorithm_time)
