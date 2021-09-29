import string
from typing import Dict, List

from revolve2.abstract.sequential_identifier import PopulationIdentifier
from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.age import GenerationalAge
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.ecology.metrics import PopulationMetrics, IndividualMetrics
from revolve2.analysis.statistics import MeasurementStatistics


class Reporter:

    def __init__(self):
        pass

    def add(self):
        pass


class Population:

    identifier = PopulationIdentifier()

    def __init__(self, individuals: Actors, rejected_individuals: Actors = None):
        self.id: int = self.identifier.id()
        self.age: GenerationalAge = GenerationalAge()
        self.history: List[Dict[int, Individual]] = []

        self.individuals: Actors = individuals
        self.rejected_individual: Actors = rejected_individuals if not(rejected_individuals is None) else []

        self.generational_statistics: MeasurementStatistics = MeasurementStatistics(['fitness', 'age'])

        self.population_metrics: PopulationMetrics = PopulationMetrics()
        self.individual_metrics: IndividualMetrics = IndividualMetrics()

        self._update_history()

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

        self._update_history()
        #self._update_history(rejected_agents)

    def _update_history(self):
        generation_history: Dict[int, Individual] = {}

        for individual in self.individuals:
            if individual.id not in generation_history:
                generation_history[individual.id] = individual

        self.history.append(generation_history)

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
