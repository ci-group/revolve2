from nca.core.genome.representations import \
    LSystemRepresentation
from nca.revolve.robot.morphology.body.robogen import RobogenAlphabet
from nca.revolve.robot.morphology.body.robogen import RobogenRules
from nca.revolve.robot.morphology.body.robogen import \
    RobogenMutationOperator, RobogenRecombinationOperator


class RobogenRepresentation(LSystemRepresentation):

    def __init__(self, alphabet: type(RobogenAlphabet) = RobogenAlphabet, rules: RobogenRules = RobogenRules,
                 mutation: RobogenMutationOperator = RobogenMutationOperator(),
                 recombination: RobogenRecombinationOperator = RobogenRecombinationOperator()):
        super().__init__(alphabet=alphabet, rules=rules, mutation=mutation, recombination=recombination)

    def compatibility(self, other):
        pass