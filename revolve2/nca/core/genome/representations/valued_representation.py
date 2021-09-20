from abc import ABC

import numpy as np

from revolve2.abstract.configurations import RepresentationConfiguration
from revolve2.nca.core.evolution.conditions.initialization import ValuedInitialization
from revolve2.nca.core.genome.operators.initialization import UniformInitialization
from revolve2.nca.core.genome.representations.representation import Representation


class ValuedRepresentation(Representation, ABC):

    def __init__(self, initialization: ValuedInitialization = UniformInitialization(), configuration=RepresentationConfiguration()):
        super().__init__(initialization, configuration)

    def compatibility(self, other) -> float:
        return np.linalg.norm(np.array(self) - np.array(other))

    def visit(self, representation_visitor):
        representation_visitor.visit_valued_representation(self)
