from abc import abstractmethod

from nca.core.actor.age import Age
from nca.core.abstract.sequential_identifier import AgentIdentifier
from nca.core.actor.fitness import Fitness, CombinedFitness
from nca.core.genome.representation import Representation


class Actor:

    @abstractmethod
    def performance(self, fitness: Fitness):
        pass


class Individual(Actor):
    identifier = AgentIdentifier()

    def __init__(self, representation: Representation):
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        self.representation: Representation = representation
        self.fitness: CombinedFitness = CombinedFitness()


    def __lt__(self, other):
        return self.id < other.id

    def performance(self, fitness: Fitness):
        self.fitness.add(fitness(self))
        self.fitness = self.fitness()  # calculate final/intermediate average

    def __repr__(self):
        return str(self.id) + " " + str(self.representation) + " " + str(self.fitness) + "\n"
