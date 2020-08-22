from abc import abstractmethod

from nca.core.abstract.creational.factory import Factory
from nca.core.agent.individual import Agent, Individual
from nca.core.agent.agents import Agents
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import UniformInitialization, BinaryInitialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.robot.brain.brain import Brain


class ActorFactory(Factory):

    def __init__(self):
        super().__init__()
        self.initialization: Initialization = None

    @abstractmethod
    def create(self, number_of_robots: int) -> Agents:
        pass

    def initialize(self, initialization: Initialization = UniformInitialization()):
        self.initialization: Initialization = initialization
        return self


class IndividualFactory(ActorFactory):

    def __init__(self, representation_type: type(Representation) = Representation):
        super().__init__()
        self.representation_type: type(Representation) = representation_type


    def create(self, number_of_individuals: int) -> Agents:
        return Agents([Individual(self.representation_type().initialize(self.initialization)) for _ in range(number_of_individuals)])


class AgentFactory(ActorFactory):

    def __init__(self, representation_type: type(Representation) = Representation):
        super().__init__()
        self.representation_type: type(Representation) = representation_type

    def create(self, number_of_agents: int) -> Agents:
        return Agents([Agent(Brain(self.representation_type().initialize(self.initialization))) for _ in range(number_of_agents)])


if __name__ == "__main__":
    factory = AgentFactory(ValuedRepresentation)
    factory.initialize(BinaryInitialization())

    individuals = factory.create(3)

    print(individuals)
