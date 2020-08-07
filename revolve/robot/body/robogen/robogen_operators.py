from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.operators.mutation_operator import MutationOperator
from nca.core.genome.operators.recombination_operator import RecombinationOperator
from nca.core.genome.representation import Representation
from revolve.robot.morphology import RobogenRepresentation


class RobogenInitialization(Initialization):

    def __init__(self):
        super().__init__()



class RobogenMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: Representation, initialization: Initialization):
        pass


class RobogenRecombination(RecombinationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation):
        pass
