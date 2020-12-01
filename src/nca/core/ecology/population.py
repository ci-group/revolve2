import string

from nca.core.abstract.sequential_identifier import PopulationIdentifier
from nca.core.actor.age import GenerationalAge
from nca.core.actor.actors import Actors
from visualization.analysis.statistics import Statistics


class Population:

    identifier = PopulationIdentifier()

    def __init__(self, individuals: Actors, rejected_individuals: Actors):
        self.id: int = self.identifier.id()
        self.age: GenerationalAge = GenerationalAge()

        self.individuals: Actors = individuals
        self.rejected_individual: Actors = rejected_individuals
        self.statistics: Statistics = Statistics()

    def __get__(self):
        return self.individuals

    def __len__(self):
        return len(self.individuals)

    def next_generation(self, agents: Actors):
        if agents is None:
            raise Exception("Empty new generation")

        self.age.increment(self.did_improve(agents))
        agents.update_age()

        # sort agents
        self.individuals = agents

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
            'fitness': self.individuals.fitness_statistics(),
        }

