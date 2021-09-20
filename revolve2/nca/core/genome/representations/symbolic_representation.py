
from revolve2.nca.core.evolution.conditions.initialization import SymbolicInitialization
from revolve2.nca.core.genome.representations.representation import Representation


# Or grammar Representation
class SymbolicRepresentation(Representation):

    def __init__(self, symbol):
        super().__init__(SymbolicInitialization(symbol))

    def compatibility(self, other) -> float:
        differences = 0

        for index, element in enumerate(self):
            if self[index] != other[index]:
                differences += 1

        return differences

    def visit(self, representation_visitor):
        representation_visitor.visit_symbolic_representation(self)

    def random_value(self):
        return self.initialization(1)
