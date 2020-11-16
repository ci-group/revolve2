from nca.core.genome.genotype import Genotype
from nca.core.genome.grammar.lindenmayer_system import LSystemGenotype, SelfOrganizingLSystemGenotype
from nca.core.genome.representations.symbolic_representation import SymbolicRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.robogen.robogen_initialization import RobogenInitialization


class SelfOrganizingRobogenGenotype(SelfOrganizingLSystemGenotype):

    def __init__(self, robogen_initialization: RobogenInitialization = RobogenInitialization()):
        super().__init__(initialization=robogen_initialization)


class IndirectRobogenGenotype(LSystemGenotype):

    def __init__(self, robogen_initialization: RobogenInitialization = RobogenInitialization()):
        super().__init__(initialization=robogen_initialization)


class DirectRobogenGenotype(Genotype):

    def __init__(self):
        super().__init__(None)
        self.encoding = []
        self.initialize()

    def initialize(self):
        self.clear()
        self._initialize_single_representation(SymbolicRepresentation(RobogenSymbol), 'symbols')
        self.encoding = self['symbols']

    def __call__(self):
        return self.encoding
