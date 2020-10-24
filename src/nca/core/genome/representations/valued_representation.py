from abc import ABC

import numpy as np

from nca.core.evolution.conditions.initialization import ValuedInitialization
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.representation import Representation


class ValuedRepresentation(Representation, ABC):

    def __init__(self, initialization: ValuedInitialization = UniformInitialization()):
        super().__init__(initialization)

    def compatibility(self, other) -> float:
        return np.linalg.norm(np.array(self) - np.array(other))

    def visit(self, representation_visitor):
        representation_visitor.visit_valued_representation(self)
