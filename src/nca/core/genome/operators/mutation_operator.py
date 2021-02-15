import random
from abc import abstractmethod, ABC
from typing import Tuple

import numpy as np

from abstract.configurations import OperatorConfiguration

from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.genotype import Genotype
from nca.core.genome.operators.initialization import UniformInitialization, GaussianInitialization, BinaryInitialization, \
    IntegerInitialization
from nca.core.genome.representations.representation import Representation
from nca.core.genome.representations.tree_representation import TreeRepresentation, CoordinateTreeRepresentation
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class MutationOperator(ABC):

    def __init__(self):
        self.configuration = OperatorConfiguration()
        pass

    def __call__(self, genotype: Genotype, debug=False):
        #genotype = copy.deepcopy(genotype)  # Recombination already creates an unique genotype
        for key in genotype.keys():
            # check if we do NOT mutate
            if random.random() > self.configuration.mutation_probability and not debug:
                continue

            self._mutate(genotype.get_random_representation(key))

        return genotype

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
        choices: Tuple = tuple(representation.selection_indexes(k=2))
        representation.swap_indexes(choices)


class InsertMutation(StructuralMutation):

    def __init__(self):
        super().__init__()

    def _mutate(self, representation: Representation):
        index = representation.random_index()
        representation[:] = np.insert(representation, index, representation.random_value())


class ReplaceMutation(StructuralMutation):

    def _mutate(self, representation: Representation):
        index = representation.random_index()
        representation[index] = representation.random_value()


class UniqueReplaceMutation(StructuralMutation):

    def _mutate(self, representation: Representation):
        index = representation.random_index()

        while True:
            new_value = representation.random_value()
            if new_value not in representation:
                break

        representation[index] = new_value


class DeleteMutation(StructuralMutation):

    def _mutate(self, representation: Representation):
        #if isinstance(representation, ChromosomalRepresentation):
        #    raise Exception("Unsupported mutation for ChromosomalRepresenation")

        index = representation.random_index()
        representation[:] = np.delete(representation, index)


class DeleteSubtreeMutation(StructuralMutation):

    def _mutate(self, representation: CoordinateTreeRepresentation):
        parent, child, orientation = representation.random_element()
        del parent.children[orientation]


class DuplicateSubtreeMutation(StructuralMutation):

    def _mutate(self, representation: CoordinateTreeRepresentation):
        parent1, child1, orientation1 = representation.random_element()

        parent2, child2, orientation2 = representation.random_element()

        del parent1[orientation1]
        del child2[orientation2.opposite()]

        parent1[orientation1] = child2


class NoMutation(StructuralMutation):

    def _mutate(self, representation: Representation):
        pass


class InversionMutation(StructuralMutation):

    def _mutate(self, representation: Representation):
        selection_range = representation.range_selection()
        representation.swap_indexes(selection_range)


class DisplacementMutation(StructuralMutation):

    def _mutate(self, representation: Representation):
        choices = representation.selection_indexes()
        selection_range = representation.range_selection()

        index = representation.random_index()
        #TODO
        #return ...

