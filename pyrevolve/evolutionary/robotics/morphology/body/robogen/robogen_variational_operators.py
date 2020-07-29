from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.robotics.morphology.body.robogen.robogen_representation import RobogenRepresentation


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
