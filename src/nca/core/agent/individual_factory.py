from abc import abstractmethod

from nca.core.abstract.factory import Factory
from nca.core.agent.individual import Agent, Individual
from nca.core.agent.agents import Agents
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import BinaryRepresentation
from revolve.robot.brain.brain import Brain


class ActorFactory(Factory):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def create(self, number_of_robots: int) -> Agents:
        pass


class IndividualFactory(ActorFactory):

    def __init__(self, representation_type: type(Representation) = Representation):
        super().__init__()
        self.representation_type: type(Representation) = representation_type

    def create(self, number_of_individuals: int) -> Agents:
        return Agents([Individual(self.representation_type()) for _ in range(number_of_individuals)])


class AgentFactory(ActorFactory):

    def __init__(self, representation_type: type(Representation) = Representation):
        super().__init__()
        self.representation_type: type(Representation) = representation_type

    def create(self, number_of_agents: int) -> Agents:
        return Agents([Agent(Brain(self.representation_type())) for _ in range(number_of_agents)])


if __name__ == "__main__":
    factory = AgentFactory(BinaryRepresentation)

    individuals = factory.create(3)

    print(individuals)
