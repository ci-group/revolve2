from abc import abstractmethod
from typing import Dict

from nca.core.abstract.creational.factory import Factory
from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.genome.genotype import Genotype
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.robot.robot import Robot


class Characterization:

    def __init__(self, representation_type: type(Representation) = ValuedRepresentation,
                 initialization_type: type(Initialization) = UniformInitialization):
        self.representation_type: type(Representation) = representation_type
        self.initialization_type: type(Initialization) = initialization_type


class ActorFactory(Factory):

    def __init__(self, characterizations = None, actor_type: type(Individual) = Individual):
        super().__init__()
        self.actor_type = actor_type

        if characterizations is None:
            characterizations = {'default': Characterization()}
        elif isinstance(characterizations, Characterization):
            characterizations = {'default': characterizations}
        self.characterizations: Dict[str, Characterization] = characterizations

    def create_genotype(self):
        representations = []
        keys = []
        for character_key in self.characterizations.keys():
            characterization = self.characterizations[character_key]
            if characterization.initialization_type is None:
                raise Exception(str(self.__class__) + " Factory not initialized yet")
            keys.append(character_key)
            representations.append(characterization.representation_type(
                characterization.initialization_type()))

        genotype: Genotype = Genotype(representations, keys)

        return genotype

    @abstractmethod
    def create(self, number_of_actors: int) -> Actors:
        return Actors([self.actor_type(self.create_genotype()) for _ in range(number_of_actors)])

    def initialize(self, initialization_type: type(Initialization)):
        self.initialization_type = initialization_type

