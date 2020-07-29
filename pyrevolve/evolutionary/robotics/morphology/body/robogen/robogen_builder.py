from pyrevolve.evolutionary.algorithm.genome.representation_builder import RepresentationBuilder
from pyrevolve.evolutionary.robotics.morphology.body.robogen.robogen_representation import RobogenRepresentation
from pyrevolve.evolutionary.robotics.morphology.body.robogen.robogen_variational_operators import \
    RobogenMutationOperator, RobogenRecombinationOperator


class RobogenBuilder(RepresentationBuilder):

    def __init__(self):
        super().__init__(mutation=RobogenMutationOperator(), recombination=RobogenRecombinationOperator())

    def generate(self):
        return RobogenRepresentation()

