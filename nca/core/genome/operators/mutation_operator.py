import random
from abc import abstractmethod

import numpy as np

from nca.experiment.configurations import MutationConfiguration
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.direct_representation import DirectRepresentation, \
    BinaryRepresentation


class MutationOperator:

    def __init__(self):
        self.configuration = MutationConfiguration()
        pass

    def algorithm(self, representation: Representation, initialization: Initialization):
        # check if we do have to do the mutation
        if self.configuration.mutation_probability > np.random.random():
            self._execute(representation, initialization)

        return representation

    @abstractmethod
    def _execute(self, representation: Representation, initialization: Initialization):
        pass


class SwapMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation, initialization: Initialization):
        choices = representation.selection_indexes()
        representation.swap_indexes(choices)


class InsertMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation, initialization: Initialization):
        index = random.choice(range(len(representation.genome)))
        representation.genome = np.insert(representation.genome, index, initialization.algorithm(1))


class ReplaceMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation, initialization: Initialization):
        index = random.choice(range(len(representation.genome)))
        representation.genome[index] = initialization.algorithm(1)


class InversionMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation, initialization: Initialization):
        selection_range = representation.range_selection()
        representation.swap_indexes(selection_range)


class DisplacementMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation, initialization: Initialization):
        choices = representation.selection_indexes()
        selection_range = representation.range_selection()

        shift_index = np.random.choice(range(len(representation.genome)))
        #TODO
        #return ...


class BitFlipMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: BinaryRepresentation, initialization: Initialization):
        index = random.choice(range(len(representation.genome)))

        representation.genome[index] = not representation.genome[index]
