from typing import List, Dict

from revolve2.nca.core.actor.age import Age
from revolve2.abstract.sequential_identifier import AgentIdentifier
from revolve2.nca.core.actor.fitness import Fitness
from revolve2.nca.core.actor.measures import Measurement
from revolve2.nca.core.genome.genotype import Genotype


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, mapping, parents: List = None):
        self.id: int = self.identifier.id()
        self.age: Age = Age()
        self.genotype: Genotype = Genotype.check(mapping)
        self.fitness: Fitness = Fitness()
        self.measures: Dict[Measurement, List] = dict()

        if parents is not None:
            self.parent_ids: List[int] = [parent.id for parent in parents]
        else:
            self.parent_ids: List[int] = []

    def __lt__(self, other):
        return self.id < other.id

    def get_fitness(self):
        return self.fitness.value()

    def get_objectives(self) -> List[float]:
        return list(self.fitness.objectives.values())

    def get_representation(self, key=None):
        if len(self.genotype.keys()) == 1:
            return self.genotype[list(self.genotype.keys())[0]]

        if key is None:
            return self.genotype
        elif key in self.genotype:
            return self.genotype[key]

        raise Exception("Key not in genotype to get representation.")

    def __repr__(self):
        return str(self.id) + " " + str(self.genotype) + " " + str(self.fitness) + "\n"
