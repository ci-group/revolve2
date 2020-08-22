from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.operators.mutation_operator import MutationOperator


class RepresentationVisitor:

    def __init__(self, initialization: Initialization, mutation: MutationOperator):
        self.initialization: Initialization = initialization
        self.mutation: MutationOperator = mutation

    def mutate_valued_representation(self, valued_representation):
        valued_representation = self.mutation.algorithm(valued_representation, self.initialization)

    def mutate_grammar_representation(self, grammar_representation):
        grammar_representation = self.mutation.algorithm(grammar_representation, self.initialization)

    def initialize_integer_representation(self, valued_representation):
        valued_representation = self.mutation.algorithm(valued_representation, self.initialization)

    def mutate_grammar_representation(self, grammar_representation):
        self.mutation.algorithm(grammar_representation, self.initialization)
