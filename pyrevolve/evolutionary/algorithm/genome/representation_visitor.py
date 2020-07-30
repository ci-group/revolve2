from abc import abstractmethod

from pyrevolve.evolutionary.algorithm.conditions.initialization import Initialization


class RepresentationVisitor:

    def __init__(self):
        pass

    @abstractmethod
    def visit_valued_representation(self, valued_representation):
        pass

    @abstractmethod
    def visit_grammar_representation(self, grammar_representation):
        pass


class InitializationVisitor(RepresentationVisitor):

    def __init__(self, initialization: Initialization):
        super().__init__()
        self.initialization: Initialization = initialization

    def visit_valued_representation(self, valued_representation):
        new_genome = self.initialization.algorithm(len(valued_representation.genome))
        valued_representation.genome = valued_representation.data_type(new_genome)

    def visit_grammar_representation(self, grammar_representation):
        new_genome = self.initialization.algorithm(len(grammar_representation.genome), len(grammar_representation.alphabet.list()))
        grammar_representation.genome = grammar_representation.data_type(new_genome)


class MutationVisitor(RepresentationVisitor):

    def __init__(self, mutation):
        super().__init__()
        self.mutation = mutation

    def visit_valued_representation(self, valued_representation):
        self.mutation.algorithm(valued_representation)

    def visit_grammar_representation(self, grammar_representation):
        self.mutation.algorithm(grammar_representation)
