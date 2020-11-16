from typing import List

from nca.core.actor.age import Age
from nca.core.abstract.sequential_identifier import AgentIdentifier
from nca.core.actor.fitness import Fitness
from nca.core.genome.genotype import Genotype


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, mapping, parents: List = None):
        self.id: int = self.identifier.id()
        self.age: Age = Age()
        self.genotype: Genotype = Genotype.check(mapping)
        self.fitness: Fitness = Fitness()

        if parents is not None:
            self.parent_ids: List[int] = [parent.id for parent in parents]
        else:
            self.parent_ids: List[int] = None

    def __lt__(self, other):
        return self.id < other.id

    def get_fitness(self):
        return self.fitness.value()

    def get_representation(self):
        if len(self.genotype.keys()) == 1:
            return self.genotype[list(self.genotype.keys())[0]]
        else:
            return self.genotype

    def __repr__(self):
        return str(self.id) + " " + str(self.genotype) + " " + str(self.fitness) + "\n"
