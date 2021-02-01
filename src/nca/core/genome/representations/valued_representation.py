from abc import ABC

import numpy as np

from nca.core.abstract.configurations import RepresentationConfiguration
from nca.core.evolution.conditions.initialization import ValuedInitialization
from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.representations.representation import Representation


class ValuedRepresentation(Representation, ABC):

    def __init__(self, initialization: ValuedInitialization = UniformInitialization(), configuration=RepresentationConfiguration()):
        super().__init__(initialization, configuration)

    def compatibility(self, other) -> float:
        return np.linalg.norm(np.array(self) - np.array(other))

    def visit(self, representation_visitor):
        representation_visitor.visit_valued_representation(self)
