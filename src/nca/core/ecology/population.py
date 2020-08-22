import string

from nca.core.abstract.sequential_identifier import PopulationIdentifier
from nca.core.agent.age import GenerationalAge
from nca.core.agent.agents import Agents


class Population:

    identifier = PopulationIdentifier()

    def __init__(self, individuals: Agents):
        self.id: int = self.identifier.id()
        self.age: GenerationalAge = GenerationalAge()

        self.individuals: Agents = individuals

    def __get__(self):
        return self.individuals

    def next_generation(self, agents: Agents):
        if agents is None:
            raise Exception("Empty new generation")

        self.age.increment(self.did_improve(agents))

        agents.update_age()

        self.individuals = agents

    def did_improve(self, agents: Agents):
        return agents.average_fitness() > self.individuals.average_fitness()

    def __repr__(self):
        string_representation: string = "Population {"

        for individual in self.individuals:
            string_representation += repr(individual) + ","

        string_representation += "}"

        return string_representation
