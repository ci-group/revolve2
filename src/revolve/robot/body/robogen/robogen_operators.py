import random

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.operators.mutation_operator import MutationOperator
from nca.core.genome.operators.recombination_operator import RecombinationOperator
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol, RobogenModule
from revolve.robot.body.robogen.robogen_representation import RobogenRepresentation


class RobogenInitialization(Initialization):

    def __init__(self):
        super().__init__()

    def algorithm(self, size: int):
        return random.choices(RobogenSymbol.alphabet(), weights=RobogenSymbol.probabilities(), k=size)


class RobogenMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation, initialization: RobogenInitialization):
        pass


class RobogenSwapMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation, initialization: RobogenInitialization):
        choices = representation.selection_indexes()
        representation.swap_indexes(choices)


class RobogenInsertMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation, initialization: Initialization):
        index = random.choice(range(len(representation.genome)))

        #TODO
        new_coordinate = representation.genome[index].coordinate
        representation.genome.insert(index, RobogenModule(initialization.algorithm(1)[0]))


class RobogenReplaceMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation, initialization: Initialization):
        index = random.choice(range(len(representation.genome)))
        representation.genome[index] = RobogenModule(initialization.algorithm(1)[0],
                                                     representation.genome[index].coordinate)


class RobogenInversionMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation, initialization: Initialization):
        selection_range = representation.range_selection()
        # TODO check same module
        representation.swap_indexes(selection_range)


class RobogenRecombination(RecombinationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: RobogenRepresentation):
        pass
