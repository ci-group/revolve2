from abc import abstractmethod

from nca.core.abstract.creational.factory import Factory
from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class IndividualFactory(Factory):

    def __init__(self, representation_type: type(Representation) = ValuedRepresentation,
                 initialization_type: type(Initialization) = UniformInitialization):
        super().__init__()
        self.representation_type: type(Representation) = representation_type
        self.initialization_type: type(Initialization) = initialization_type

    @abstractmethod
    def create(self, number_of_actors: int) -> Actors:
        if self.initialization_type is None:
            raise Exception(str(self.__class__) + " Factory not initialized yet")
        return Actors([Individual(self.representation_type(self.initialization_type())) for _ in range(number_of_actors)])

    def initialize(self, initialization_type: type(Initialization)):
        self.initialization_type = initialization_type

