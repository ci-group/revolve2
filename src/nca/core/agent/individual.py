from typing import Dict

from nca.core.agent.age import Age
from nca.core.agent.fitness import Fitness
from nca.core.abstract.sequential_identifier import AgentIdentifier


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, representation: Dict, fitness: Fitness):
        super().__init__()
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        self.representation = representation

        self.fitness: Fitness = fitness

    def __lt__(self, other):
        return self.id < other.id