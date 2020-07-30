from pyrevolve.shared.sequential_identifier import PopulationIdentifier
from pyrevolve.evolutionary.age import GenerationalAge

from .. import Agents


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
        self.offspring = None

    def did_improve(self, agents: Agents):
        return agents.average_fitness() > self.individuals.average_fitness()
