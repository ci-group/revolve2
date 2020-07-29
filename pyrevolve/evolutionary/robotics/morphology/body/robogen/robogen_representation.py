from pyrevolve.evolutionary.algorithm.genome.representations.l_system.lsystem_representation import \
    LSystemRepresentation
from pyrevolve.evolutionary.robotics.morphology.body.robogen.robogen_alphabet import RobogenAlphabet
from pyrevolve.evolutionary.robotics.morphology.body.robogen.robogen_rules import RobogenRules
from pyrevolve.evolutionary.robotics.morphology.body.robogen.robogen_variational_operators import \
    RobogenMutationOperator, RobogenRecombinationOperator


class RobogenRepresentation(LSystemRepresentation):

    def __init__(self, alphabet: type(RobogenAlphabet) = RobogenAlphabet, rules: RobogenRules = RobogenRules,
                 mutation: RobogenMutationOperator = RobogenMutationOperator(),
                 recombination: RobogenRecombinationOperator = RobogenRecombinationOperator()):
        super().__init__(alphabet=alphabet, rules=rules, mutation=mutation, recombination=recombination)

    def compatibility(self, other):
        pass