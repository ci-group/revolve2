
from nca.core.actor.age import Age
from nca.core.abstract.sequential_identifier import AgentIdentifier
from nca.core.actor.fitness import Fitness
from nca.core.genome.genotype import Genotype
from nca.core.genome.representation import Representation


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, genotype):
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        if isinstance(genotype, Representation):
            genotype = Genotype(genotype)

        if not isinstance(genotype, Genotype):
            raise Exception("Genotype failure in individual")

        self.genotype: Genotype = genotype
        self.fitness: Fitness = Fitness()

    def __lt__(self, other):
        return self.id < other.id

    def get_fitness(self):
        return self.fitness.value()

    def get_fitnesses(self):
        return self.fitness.values()

    def get_representation(self):
        if len(self.genotype.keys()) == 1:
            return self.genotype[list(self.genotype.keys())[0]]
        else:
            return self.genotype

    def __repr__(self):
        return str(self.id) + " " + str(self.genotype) + " " + str(self.fitness) + "\n"
