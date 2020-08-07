from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.conditions.special_features import SpecialFeatures
from nca.core.evolution.conditions.termination_condition import EvaluationsCondition, TerminationCondition
from nca.core.evolution.evolutionary_configurations import EvolutionConfiguration
from nca.core.evolution.selection.parent_selection import RouletteWheelSelection
from nca.core.evolution.selection.selection import ParentSelection, SurvivorSelection
from nca.core.evolution.selection.survivor_selection import GenerationalSteadyStateSelection
from nca.core.genome.initialization import UniformInitialization
from nca.core.genome.representations.direct_representation import BinaryRepresentation


class GeneticAlgorithmConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: BinaryRepresentation = BinaryRepresentation(),
                 recombination: RobogenRecombination = RobogenRecombination(),
                 mutation: RobogenMutation = RobogenMutation(),
                 parent_selection: ParentSelection = RouletteWheelSelection(),
                 survivor_selection: SurvivorSelection = GenerationalSteadyStateSelection(),
                 initialization: Initialization = UniformInitialization(),
                 termination_condition: TerminationCondition = EvaluationsCondition(10),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, parent_selection, survivor_selection,
            initialization, termination_condition, special_features)
