from nca.core.genome.operators.mutation_operator import MutationOperator
from nca.core.genome.operators.recombination_operator import RecombinationOperator
from nca.revolve.robot.morphology.body.robogen import RobogenRepresentation


class RobogenMutationOperator(MutationOperator):

    def __init__(self):
        super().__init__()

    def algorithm(self, representation: RobogenRepresentation):
        pass


class RobogenRecombinationOperator(RecombinationOperator):

    def __init__(self):
        super().__init__()

    def algorithm(self, representation: RobogenRepresentation):
        pass
