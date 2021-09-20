import copy
from abc import abstractmethod

from revolve2.abstract.creational.factory import Factory
from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.evolution.conditions.initialization import Initialization
from revolve2.nca.core.genome.genotype import Genotype
from revolve2.nca.core.genome.operators.initialization import UniformInitialization
from revolve2.nca.core.genome.representations.representation import Representation
from revolve2.nca.core.genome.representations.valued_representation import ValuedRepresentation


class Characterization:

    def __init__(self, representation_type: type(Representation) = ValuedRepresentation,
                 initialization_type: type(Initialization) = UniformInitialization):
        self.representation_type: type(Representation) = representation_type
        self.initialization_type: type(Initialization) = initialization_type


class ActorFactory(Factory):

    def __init__(self, mapping=None, actor_type: type(Individual) = Individual):
        super().__init__()
        self.actor_type = actor_type

        if mapping is None:
            mapping = ValuedRepresentation()
        self.genotype = Genotype.check(mapping)

    def create_genotype(self):
        genotype: Genotype = copy.deepcopy(self.genotype)
        genotype.initialize()
        return genotype

    @abstractmethod
    def create(self, number_of_actors: int) -> Actors:
        return Actors([self.actor_type(self.create_genotype()) for _ in range(number_of_actors)])
