import string
from enum import Enum
from typing import List, Dict

from nca.core.abstract.sequential_identifier import PopulationIdentifier
from nca.core.actor.actors import Actors
from nca.core.actor.age import GenerationalAge
from nca.core.actor.measures import BehavioralMeasurement, MorphologicalMeasurement, EpigeneticMeasurement, Measurement
from visualization.analysis.statistics import Statistics, MeasurementStatistics


class Reporter:

    def __init__(self):
        pass

    def add(self):
        pass


class Metrics:
    def __init__(self):
        pass


class IndividualMetrics:

    def __init__(self):
        self.measures: Dict[Measurement, List] = dict()

        self.initialize()

    def initialize(self):
        for measurement_type in [BehavioralMeasurement, EpigeneticMeasurement, MorphologicalMeasurement]:
            for measurement in measurement_type:
                self.measures[measurement] = []

    def log(self, individuals):
        for individual in individuals:
            for measure_key in individual.measures.keys():
                self.measures[measure_key].append(individual.measures[measure_key])


class PopulationMetrics(Dict[str, List]):

    def __init__(self):
        super().__init__()
        self["rejection_rate"] = []
        self["acceptance_rate"] = []
        self.old_individual_ids = []

    def log(self, individuals, rejected_individuals):
        total_individuals: int = len(individuals)
        total_offspring: int = len(rejected_individuals)

        count_rejection: int = 0
        count_acceptance: int = 0

        for individual in individuals:
            if individual.id not in self.old_individual_ids:
                count_acceptance += 1
                self.old_individual_ids.append(individual.id)

        for individual in rejected_individuals:
            if individual.id in self.old_individual_ids:
                self.old_individual_ids.remove(individual.id)
                count_rejection += 1

        self["acceptance_rate"].append(count_acceptance / total_individuals)
        if total_offspring > 0:
            self["rejection_rate"].append(count_rejection / total_offspring)
        else:
            self["rejection_rate"].append(0.0)

    def __repr__(self):
        output = ""
        output += "rejection: " + str(self["rejection_rate"]) + "\n"
        output += "acceptance: " + str(self["acceptance_rate"]) + "\n"
        return output


class Population:

    identifier = PopulationIdentifier()

    def __init__(self, individuals: Actors, rejected_individuals: Actors = None):
        self.id: int = self.identifier.id()
        self.age: GenerationalAge = GenerationalAge()

        self.individuals: Actors = individuals
        self.rejected_individual: Actors = rejected_individuals if not(rejected_individuals is None) else []

        self.generational_statistics: MeasurementStatistics = MeasurementStatistics(['fitness', 'age'])

        self.population_metrics: PopulationMetrics = PopulationMetrics()
        self.individual_metrics: IndividualMetrics = IndividualMetrics()

    def __get__(self):
        return self.individuals

    def __len__(self):
        return len(self.individuals)

    def next_generation(self, new_agents: Actors, rejected_agents: Actors):
        if new_agents is None:
            raise Exception("Empty new generation")

        self.age.increment(self.did_improve(new_agents))
        new_agents.update_age()

        # sort agents
        self.individuals = new_agents
        self.rejected_individual = rejected_agents

    def did_improve(self, agents: Actors):
        return agents.average_fitness() > self.individuals.average_fitness()

    def __repr__(self):
        string_representation: string = "Population {"

        for individual in self.individuals:
            string_representation += repr(individual) + ","

        string_representation += "}"

        return string_representation

    def to_json(self):
        return {
            'id': self.id,
            'age': self.age.no_improvement_count,
            'individuals': [individual.id for individual in self.individuals],
            'typical': self.individuals.find_typical_individuals(),
            'fitness': self.generational_statistics.latest(),
        }

    def log(self):
        self.generational_statistics['fitness'].log_values(values=[individual.fitness.value() for individual in self.individuals])
        self.generational_statistics['age'].log_values(values=[individual.age.generations for individual in self.individuals])
        self.population_metrics.log(self.individuals, self.rejected_individual)
        self.individual_metrics.log(self.individuals)
