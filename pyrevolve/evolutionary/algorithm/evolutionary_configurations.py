from pyrevolve.evolutionary.algorithm.genome.operators.mutation.mutation_operator import MutationOperator
from pyrevolve.evolutionary.algorithm.genome.operators.recombination.recombination_operator import RecombinationOperator
from pyrevolve.evolutionary.algorithm.genome.representation import Representation
from pyrevolve.evolutionary.algorithm.genome.representations.direct.binary_representation import BinaryRepresentation
from pyrevolve.evolutionary.algorithm.genome.representations.direct.real_valued_representation import \
    RealValuedRepresentation
from pyrevolve.evolutionary.algorithm.genome.representations.tree_representation import TreeRepresentation
from pyrevolve.evolutionary.algorithm.conditions.initialisation import Initialisation
from pyrevolve.evolutionary.algorithm.selection.parent_selection import RouletteWheelSelection
from pyrevolve.evolutionary.algorithm.selection.selection import ParentSelection, SurvivorSelection
from pyrevolve.evolutionary.algorithm.selection.survivor_selection import ElitismSelection
from pyrevolve.evolutionary.algorithm.conditions.special_features import SpecialFeatures
from pyrevolve.evolutionary.algorithm.conditions.termination_condition import TerminationCondition
from pyrevolve.shared.configuration import Configuration


class EvolutionConfiguration(Configuration):

    def __init__(self,
                 representation: Representation,
                 recombination: RecombinationOperator,
                 mutation: MutationOperator,
                 mutation_probability: float,
                 parent_selection: ParentSelection,
                 survivor_selection: SurvivorSelection,
                 population_size,
                 initialisation: Initialisation,
                 termination_condition: TerminationCondition,
                 special_features: SpecialFeatures):

        super().__init__("algorithm.config")
        self.representation: Representation = representation
        self.recombination: RecombinationOperator = recombination
        self.mutation: MutationOperator = mutation
        self.mutation_probability: float = mutation_probability
        self.parent_selection: ParentSelection = parent_selection
        self.survivor_selection: SurvivorSelection = survivor_selection
        self.population_size: int = population_size
        self.initialisation: Initialisation = initialisation
        self.termination_condition: TerminationCondition = termination_condition
        self.special_features: SpecialFeatures = special_features


class GeneticAlgorithmConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: BinaryRepresentation = BinaryRepresentation(),
                 recombination: nPointCrossover = nPointCrossover(1),
                 mutation: BitFlipMutation = BitFlipMutation(),
                 mutation_probability: float = 0.7,
                 parent_selection: ParentSelection = RouletteWheelSelection("fitness"),
                 survivor_selection: SurvivorSelection = GenerationalSelection(),
                 population_size: int = 20,
                 initialisation: Initialisation = Initialisation(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, mutation_probability,
            parent_selection, survivor_selection, population_size,
            initialisation, termination_condition, special_features)


class EvolutionaryStrategiesConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation=RealValuedRepresentation(),
                 recombination=DiscreteCrossover(),
                 mutation=GaussianMutation(),
                 mutation_probability: float = 0.7,
                 parent_selection : ParentSelection = UniformRandomSelection(),
                 survivor_selection: SurvivorSelection = ElitismSelection(),
                 population_size: int = 20,
                 initialisation: Initialisation = Initialisation(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, mutation_probability,
            parent_selection, survivor_selection, population_size,
            initialisation, termination_condition, special_features)


class EvolutionaryProgrammingConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: Representation = RealValuedRepresentation(),
                 recombination: RecombinationOperator = None,
                 mutation: MutationOperator = GaussianMutation(),
                 mutation_probability: float = 0.7,
                 parent_selection: ParentSelection = DeterministicSelection(),
                 survivor_selection: SurvivorSelection = ProbabilisticSelection(),
                 population_size: int = 20,
                 initialisation: Initialisation = Initialisation(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, mutation_probability,
            parent_selection, survivor_selection, population_size,
            initialisation, termination_condition, special_features)


class GeneticProgrammingConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: Representation = TreeRepresentation(),
                 recombination: RecombinationOperator = SubtreesRecombination(),
                 mutation: MutationOperator = RandomTreeMutation(),
                 mutation_probability: float = 0.7,
                 parent_selection: ParentSelection = ProportionalSelection("fitness"),
                 survivor_selection: SurvivorSelection = GenerationalSelection(),
                 population_size: int = 20,
                 initialisation: Initialisation = Initialisation(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, mutation_probability,
            parent_selection, survivor_selection, population_size,
            initialisation, termination_condition, special_features)


class DifferentialEvolutionConfiguration(EvolutionConfiguration):

    def __init__(self,
                 representation: Representation = RealValuedRepresentation(),
                 recombination: RecombinationOperator = UniformCrossover(),
                 mutation: MutationOperator = DifferentialMutation(),
                 mutation_probability: float = 0.7,
                 parent_selection: ParentSelection = UniformRandomSelection(),
                 survivor_selection: SurvivorSelection = ElitismSelection(),
                 population_size: int = 20,
                 initialisation: Initialisation = Initialisation(),
                 termination_condition: TerminationCondition = TerminationCondition(),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(
            representation, recombination, mutation, mutation_probability,
            parent_selection, survivor_selection, population_size,
            initialisation, termination_condition, special_features)


class ParticleSwarmOptimisationConfiguration(EvolutionConfiguration):
    def __init__(self):
        super().__init__()