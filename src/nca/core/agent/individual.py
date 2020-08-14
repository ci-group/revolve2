import math

from nca.core.agent.age import Age
from nca.core.abstract.sequential_identifier import AgentIdentifier
from revolve.robot.brain.brain import Brain


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, representation):
        super().__init__()
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        self.representation = representation
        self.fitness: float = -math.inf

    def __lt__(self, other):
        return self.id < other.id


class Agent(Individual):
    def __init__(self, brain: Brain):
        super().__init__(brain)
