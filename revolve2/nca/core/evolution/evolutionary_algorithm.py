import copy
import time
from enum import Enum
from typing import List, Dict

from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.evolution.conditions.initialization import Initialization
from revolve2.nca.core.evolution.conditions.special_features import SpecialFeatures
from revolve2.nca.core.evolution.conditions.condition import Condition
from revolve2.nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve2.nca.core.evolution.selection.mortality_selection import NullMortalitySelection
from revolve2.nca.core.evolution.selection.selection import ParentSelection, SurvivorSelection, MortalitySelection
from revolve2.nca.core.genome.operators.mutation_operator import MutationOperator
from revolve2.nca.core.genome.operators.recombination_operator import RecombinationOperator


class Duration(Enum):
    PARENT_SELECTION = 0
    MUTATION_RECOMBINATION = 1
    EVALUATION = 2
    MORTALITY_SELECTION = 3
    SURVIVOR_SELECTION = 4


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

        self.duration: Dict[Duration, List[float]] = {}
        for duration_key in Duration:
            self.duration[duration_key] = []

    def should_terminate(self, population: Population):
        return self.termination_condition.terminate(population)

    def run(self, population: Population, evaluator):
        # (PARENT) SELECT parents

        parent_selection_time = time.time()
        parent_combinations: List[List[Individual]] = self.parent_selection.select(population.individuals)

        offspring_time = time.time()
        # RECOMBINE, then MUTATE resulting recombination to create resulting offspring
        offspring: Actors = Actors([Individual(self.mutation(self.recombination(parents)), parents)
                                    for parents in parent_combinations])

        # EVALUATE the candidates
        evaluation_time = time.time()
        evaluator(offspring)

        mortality_time = time.time()
        population.individuals = self.mortality_selection.algorithm(population.individuals, offspring)

        # (SURVIVOR) SELECT individuals for the next generation
        survivor_time = time.time()
        new_individuals, rejected_individuals = self.survivor_selection.select(population.individuals)
        population.next_generation(new_individuals, rejected_individuals)

        final_time = time.time()

        self.duration[Duration.PARENT_SELECTION].append(offspring_time - parent_selection_time)
        self.duration[Duration.MUTATION_RECOMBINATION].append(evaluation_time - offspring_time)
        self.duration[Duration.EVALUATION].append(mortality_time - evaluation_time)
        self.duration[Duration.MORTALITY_SELECTION].append(survivor_time - mortality_time)
        self.duration[Duration.SURVIVOR_SELECTION].append(final_time - survivor_time)
