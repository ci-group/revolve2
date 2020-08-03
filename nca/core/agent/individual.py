
from nca.core.genome.representation import Representation
from nca.core.agent.age import Age
from nca.core.agent.fitness import Fitness
from nca.experiment.sequential_identifier import AgentIdentifier


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, representation: Representation, fitness: Fitness):
        super().__init__()
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        self.representation = representation
        self.fitness: Fitness = fitness
