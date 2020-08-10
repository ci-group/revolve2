
from nca.core.agent.individual import Individual
from nca.core.agent.agents import Agents
from nca.core.genome.representation import Representation
from nca.core.genome.representations.direct_representation import BinaryRepresentation
from nca.core.agent.fitness import Fitness, DisplacementFitness


class IndividualFactory:

    def __init__(self, representation_type: type(Representation) = Representation, fitness_type: type(Fitness) = Fitness):
        super().__init__()

        self.representation_type: type(Representation) = representation_type
        self.fitness_type: type(Fitness) = fitness_type

    def create(self, n: int = 0) -> Agents:
        return Agents([Individual(self.representation_type(), self.fitness_type()) for _ in range(n)])


if __name__ == "__main__":
    factory = IndividualFactory(BinaryRepresentation, DisplacementFitness)

    individuals = factory.create(3)

    print(individuals)
