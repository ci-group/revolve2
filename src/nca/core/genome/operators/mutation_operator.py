import copy
import random
from abc import abstractmethod, ABC
from typing import Tuple, Dict

import numpy as np

from nca.core.abstract.configurations import OperatorConfiguration
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.initialization import UniformInitialization, GaussianInitialization, BinaryInitialization, \
    IntegerInitialization
from nca.core.genome.representation import Representation
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class MutationOperator(ABC):

    def __init__(self):
        self.configuration = OperatorConfiguration()
        pass

    def __call__(self, representations: Dict[str, Representation]):
        # check if we do have to do the mutation
        new_representations = copy.deepcopy(representations)
        chosen_representation = np.random.choice(list(representations.keys()))

        self._mutate(new_representations[chosen_representation])

        return new_representations

    @abstractmethod
    def compatibility(self, initialization_type: type(Initialization)):
        pass

    @abstractmethod
    def _mutate(self, args):
        pass


class ValueMutation(MutationOperator, ABC):

    @abstractmethod
    def _mutate(self, index: int, representation: ValuedRepresentation):
        pass


class BitFlipMutation(ValueMutation):

    def __init__(self):
        super().__init__()

    def compatibility(self, initialization_type: type(Initialization)):
        if initialization_type != BinaryInitialization:
            return False
        return True

    def _mutate(self, index: int, representation: ValuedRepresentation):
        for index in range(len(representation)):
            if random.random() > self.configuration.mutation_probability:
                # TODO do premature check instead of here.
                if representation.initialization != BinaryInitialization:
                    raise Exception("Random Resetting Mutation not supported by Initialization strategy")

                representation[index] = not representation[index]


class RandomResettingMutation(ValueMutation):

    def __init__(self):
        super().__init__()

    def compatibility(self, initialization_type: type(Initialization)):
        if initialization_type != BinaryInitialization:
            return False
        return True

    def _mutate(self, index: int, representation: ValuedRepresentation):
        for index in range(len(representation)):
            if random.random() > self.configuration.mutation_probability:
                representation[index] = representation.random_value()


class RealValuedMutation(ValueMutation):

    def compatibility(self, initialization_type: type(Initialization)):
        if initialization_type in [BinaryInitialization, IntegerInitialization]:
            return False
        return True


class CreepMutation(RealValuedMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, index: int, representation: ValuedRepresentation):
        for index in range(len(representation)):
            if random.random() > self.configuration.mutation_probability:
                # TODO integer can create floats?
                representation[index] += representation.random_value() / 10  # TODO generalize scalar.


#TODO duplicate of GaussianMutation
class UniformMutation(RealValuedMutation):

    def __init__(self):
        super().__init__()
        self.random_value_generator = UniformInitialization()

    def _mutate(self, index: int, representation: ValuedRepresentation):
        random_values = self.random_value_generator(len(representation))  # TODO or not TODO
        for index in range(len(representation)):
            if random.random() > self.configuration.mutation_probability:
                representation[index] += random_values[index]


class GaussianMutation(RealValuedMutation):

    def __init__(self):
        super().__init__()
        self.random_value_generator = GaussianInitialization()

    def _mutate(self, index: int, representation: ValuedRepresentation):
        random_values = self.random_value_generator(len(representation)) # TODO or not TODO
        for index in range(len(representation)):
            if random.random() > self.configuration.mutation_probability:
                representation[index] += random_values[index]


class SelfAdaptiveMutation(RealValuedMutation):

    def __init__(self):
        super().__init__()
        self.mutation_gene_index = 0

    def _mutate(self, index: int, representation: ValuedRepresentation):
        for index in range(len(representation)):
            if random.random() > self.configuration.mutation_probability:
                raise Exception("Not implemented yet")# TODO maybe decorator option?


class StructuralMutation(MutationOperator, ABC):

    def compatibility(self, initialization_type: type(Initialization)):
        return True


class SwapMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        choices: Tuple[int] = tuple(representation.selection_indexes(k=2))
        representation.swap_indexes(choices)


class InsertMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = representation.random_index()
        representation[:] = np.insert(representation, index, representation.random_value())


class ReplaceMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = representation.random_index()
        representation[index] = representation.random_value()


class DeleteMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = representation.random_index()
        representation[:] = np.delete(representation, index)


class InversionMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        selection_range = representation.range_selection()
        representation.swap_indexes(selection_range)


class DisplacementMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        choices = representation.selection_indexes()
        selection_range = representation.range_selection()

        index = representation.random_index()
        #TODO
        #return ...
