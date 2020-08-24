import copy
import random
from abc import abstractmethod, ABC

import numpy as np

from nca.core.abstract.configurations import OperatorConfiguration
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class MutationOperator(ABC):

    def __init__(self):
        self.configuration = OperatorConfiguration()
        pass

    def __call__(self, representation: Representation):
        # check if we do have to do the mutation
        new_representation = copy.deepcopy(representation)
        if self.configuration.mutation_probability > np.random.random():
            self._mutate(new_representation)

        return new_representation

    @abstractmethod
    def _mutate(self, representation: Representation):
        pass


class LocationMutation(MutationOperator, ABC):
    pass


class ValueMutation(MutationOperator, ABC):
    pass


class SwapMutation(LocationMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        choices = representation.selection_indexes()
        representation.swap_indexes(choices)


class InsertMutation(ValueMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = representation.random_index()
        representation.genome = np.insert(representation.genome, index, representation.random_value())


class ReplaceMutation(ValueMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = random.choice(range(len(representation)))

        representation.genome[index] = representation.random_value()


class DeleteMutation(ValueMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = random.choice(range(len(representation)))
        representation.genome = np.delete(representation.genome, index)


class InversionMutation(MutationOperator):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        selection_range = representation.range_selection()
        representation.swap_indexes(selection_range)


class DisplacementMutation(LocationMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        choices = representation.selection_indexes()
        selection_range = representation.range_selection()

        shift_index = np.random.choice(range(len(representation)))
        #TODO
        #return ...


class BitFlipMutation(ReplaceMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: ValuedRepresentation):
        index = random.choice(range(len(representation)))

        representation.genome[index] = not representation.genome[index]
