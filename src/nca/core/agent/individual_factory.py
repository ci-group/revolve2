from abc import abstractmethod

from nca.core.abstract.creational.factory import Factory
from nca.core.agent.fitness import Fitness
from nca.core.agent.individual import Agent, Individual
from nca.core.agent.agents import Agents
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.representation import Representation
from revolve.robot.brain.brain import Brain, BrainRepresentation


class ActorFactory(Factory):

    def __init__(self):
        super().__init__()
        self.initialization: Initialization = None
        self.fitness: Fitness = None

    @abstractmethod
    def create(self, number_of_actors: int) -> Agents:
        if self.initialization is None:
            raise Exception(str(self.__class__) + " Factory not initialized yet")

    def initialize(self, initialization: Initialization = UniformInitialization()):
        self.initialization: Initialization = initialization
        return self


class IndividualFactory(ActorFactory):

    def __init__(self, representation_type: type(Representation) = Representation):
        super().__init__()
        self.representation_type: type(Representation) = representation_type

    def create(self, number_of_individuals: int) -> Agents:
        super().create(number_of_individuals)
        return Agents([Individual(self.representation_type(self.initialization)) for _ in range(number_of_individuals)])


class AgentFactory(ActorFactory):

    def __init__(self, representation_type: type(BrainRepresentation) = BrainRepresentation):
        super().__init__()
        self.representation_type: type(BrainRepresentation) = representation_type

    def create(self, number_of_agents: int) -> Agents:
        super().create(number_of_agents)
        return Agents([Agent(Brain(self.representation_type(self.initialization))) for _ in range(number_of_agents)])
