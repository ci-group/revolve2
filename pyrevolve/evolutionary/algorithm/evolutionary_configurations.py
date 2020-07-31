from pyrevolve.evolutionary.algorithm.genome.initialization import UniformInitialization
from pyrevolve.evolutionary.algorithm.genome.operators.mutation_operator import MutationOperator, BitFlipMutation
from pyrevolve.evolutionary.algorithm.genome.operators.recombination_operator import RecombinationOperator, \
    OnePointCrossover
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.conditions.initialization import Initialization
from pyrevolve.evolutionary.algorithm.genome.representations.direct_representation import BinaryRepresentation
from pyrevolve.evolutionary.algorithm.selection.parent_selection import RouletteWheelSelection
from pyrevolve.evolutionary.algorithm.selection.selection import ParentSelection, SurvivorSelection
from pyrevolve.evolutionary.algorithm.conditions.special_features import SpecialFeatures
from pyrevolve.evolutionary.algorithm.conditions.termination_condition import TerminationCondition, EvaluationsCondition
from pyrevolve.evolutionary.algorithm.selection.survivor_selection import GenerationalSteadyStateSelection
from pyrevolve.shared.configuration import Configuration


class EvolutionConfiguration(Configuration):

    def __init__(self,
                 representation: Representation,
                 recombination: RecombinationOperator,
                 mutation: MutationOperator,
                 parent_selection: ParentSelection,
                 survivor_selection: SurvivorSelection,
                 initialization: Initialization,
                 termination_condition: TerminationCondition,
                 special_features: SpecialFeatures):

        super().__init__("algorithm.config")
        self.representation: Representation = representation
        self.recombination: RecombinationOperator = recombination
        self.mutation: MutationOperator = mutation
        self.parent_selection: ParentSelection = parent_selection
        self.survivor_selection: SurvivorSelection = survivor_selection
        self.initialization: Initialization = initialization
        self.termination_condition: TerminationCondition = termination_condition
        self.special_features: SpecialFeatures = special_features


class GeneticAlgorithmConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: BinaryRepresentation = BinaryRepresentation(),
                 recombination: OnePointCrossover = OnePointCrossover(),
                 mutation: BitFlipMutation = BitFlipMutation(),
                 parent_selection: ParentSelection = RouletteWheelSelection(),
                 survivor_selection: SurvivorSelection = GenerationalSteadyStateSelection(),
                 initialization: Initialization = UniformInitialization(),
                 termination_condition: TerminationCondition = EvaluationsCondition(10),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, parent_selection, survivor_selection,
            initialization, termination_condition, special_features)


"""
class EvolutionaryStrategiesConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation=RealValuedRepresentation(),
                 recombination=DiscreteCrossover(),
                 mutation=GaussianMutation(),
                 parent_selection : ParentSelection = UniformRandomSelection(),
                 survivor_selection: SurvivorSelection = ElitismSelection(),
                 initialization: Initialization = Initialization(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, parent_selection, survivor_selection,
            initialization, termination_condition, special_features)


class EvolutionaryProgrammingConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: Representation = RealValuedRepresentation(),
                 recombination: RecombinationOperator = None,
                 mutation: MutationOperator = GaussianMutation(),
                 parent_selection: ParentSelection = DeterministicSelection(),
                 survivor_selection: SurvivorSelection = ProbabilisticSelection(),
                 initialization: Initialization = Initialization(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, parent_selection, survivor_selection,
            initialization, termination_condition, special_features)


class GeneticProgrammingConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: Representation = TreeRepresentation(),
                 recombination: RecombinationOperator = SubtreesRecombination(),
                 mutation: MutationOperator = RandomTreeMutation(),
                 parent_selection: ParentSelection = ProportionalSelection("fitness"),
                 survivor_selection: SurvivorSelection = GenerationalSelection(),
                 initialization: Initialization = Initialization(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, parent_selection, survivor_selection,
            initialization, termination_condition, special_features)


class DifferentialEvolutionConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: Representation = RealValuedRepresentation(),
                 recombination: RecombinationOperator = UniformCrossover(),
                 mutation: MutationOperator = DifferentialMutation(),
                 parent_selection: ParentSelection = UniformRandomSelection(),
                 survivor_selection: SurvivorSelection = ElitismSelection(),
                 initialization: Initialization = Initialization(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, parent_selection, survivor_selection,
            initialization, termination_condition, special_features)


class ParticleSwarmOptimisationConfiguration(EvolutionConfiguration):
    def __init__(self):
        super().__init__()
"""
