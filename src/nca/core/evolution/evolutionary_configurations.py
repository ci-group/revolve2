from nca.core.abstract.configuration import Configuration
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.evolution.conditions.special_features import SpecialFeatures
from nca.core.evolution.conditions.condition import Condition, EvaluationsCondition
from nca.core.evolution.selection.parent_selection import RouletteWheelSelection, TournamentSelection
from nca.core.evolution.selection.selection import ParentSelection, SurvivorSelection
from nca.core.evolution.selection.survivor_selection import FitnessSteadyStateSelection
from nca.core.genome.operators.initialization import UniformInitialization
from nca.core.genome.operators.mutation_operator import MutationOperator, ReplaceMutation
from nca.core.genome.operators.recombination_operator import RecombinationOperator, OnePointCrossover


class EvolutionaryConfiguration(Configuration):

    def __init__(self,
                 recombination: RecombinationOperator,
                 mutation: MutationOperator,
                 parent_selection: ParentSelection,
                 survivor_selection: SurvivorSelection,
                 initialization_type: type(Initialization),
                 condition: Condition,
                 special_features: SpecialFeatures):

        super().__init__("evolution.config")
        self.recombination: RecombinationOperator = recombination
        self.mutation: MutationOperator = mutation
        self.parent_selection: ParentSelection = parent_selection
        self.survivor_selection: SurvivorSelection = survivor_selection
        self.initialization_type: type(Initialization) = initialization_type
        self.condition: Condition = condition
        self.special_features: SpecialFeatures = special_features

        if not self.compatible():
            raise Exception("Algorithm not compatible with Individuals")

    def compatible(self) -> bool:
        # check population compatibility with representation.
        if not self.mutation.compatibility(self.initialization_type):
            return False

        #if not self.recombination.compatibility(self.initialization_type):
        #    return False

        return True


class GeneticAlgorithmConfiguration(EvolutionaryConfiguration):

    def __init__(self,
                 recombination: RecombinationOperator = OnePointCrossover(),
                 mutation: MutationOperator = ReplaceMutation(),
                 parent_selection: ParentSelection = TournamentSelection(),
                 survivor_selection: SurvivorSelection = FitnessSteadyStateSelection(),
                 initialization_type: type(Initialization) = UniformInitialization,
                 condition: Condition = EvaluationsCondition(500),
                 special_features: SpecialFeatures = SpecialFeatures()):
        super().__init__(recombination=recombination, mutation=mutation, parent_selection=parent_selection,
                         survivor_selection=survivor_selection, initialization_type=initialization_type,
                         condition=condition, special_features=special_features)


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
