from abc import ABC

import numpy as np

from nca.core.genome.representation import Representation


class ValuedRepresentation(Representation, ABC):

    def __init__(self):
        super().__init__()

    def compatibility(self, other) -> float:
        return np.linalg.norm(np.array(self.genome) - np.array(other.genome))

    def visit(self, representation_visitor):
        representation_visitor.visit_valued_representation(self)
