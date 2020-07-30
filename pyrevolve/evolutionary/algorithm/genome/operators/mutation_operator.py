import copy
import random
from abc import abstractmethod

import numpy as np

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import DirectRepresentation, \
    BinaryRepresentation
from pyrevolve.shared.configurations import MutationConfiguration


class MutationOperator:

    def __init__(self):
        self.configuration = MutationConfiguration()
        pass

    def algorithm(self, individual: Individual):
        # check if we do not have to do the mutation
        if self.configuration.mutation_probability < np.random.random():
            return

        self._execute(individual.representation)

    @abstractmethod
    def _execute(self, representation: Representation):
        pass


class SwapMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation):
        choices = representation.selection_indexes()
        representation.swap_indexes(choices)


class InsertMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation):
        index = random.choice(range(len(representation.genome)))
        #TODO problem
        #return np.insert(genome, index, new_value)


class ReplaceMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation):
        index = random.choice(range(len(representation.genome)))
        #TODO problem
        #representation.genome[index] = representation.initialize
        #return genome[index] = new_value


class InversionMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation):
        selection_range = representation.range_selection()
        representation.swap_indexes(selection_range)


class DisplacementMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: DirectRepresentation):
        choices = representation.selection_indexes()
        selection_range = representation.range_selection()

        shift_index = np.random.choice(range(len(representation.genome)))

        #return ...


class BitFlipMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _execute(self, representation: BinaryRepresentation):
        index = random.choice(range(len(representation.genome)))

        representation.genome[index] = not representation.genome[index]