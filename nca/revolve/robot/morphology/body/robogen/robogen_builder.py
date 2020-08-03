from nca.revolve.robot.morphology.body.robogen import RobogenRepresentation
from nca.revolve.robot.morphology.body.robogen import \
    RobogenMutationOperator, RobogenRecombinationOperator


class RobogenBuilder(RepresentationBuilder):

    def __init__(self):
        super().__init__(mutation=RobogenMutationOperator(), recombination=RobogenRecombinationOperator())

    def generate(self):
        return RobogenRepresentation()

