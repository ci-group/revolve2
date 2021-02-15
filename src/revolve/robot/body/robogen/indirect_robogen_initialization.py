import random
import numpy as np

from nca.core.genome.grammar.grammar_initialization import LSystemInitialization
from nca.core.genome.grammar.symbol import Symbol
from nca.core.genome.representations.symbolic_representation import SymbolicRepresentation
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robogen_word import RobogenWord


class RobogenInitialization(LSystemInitialization):

    def __init__(self, symbol_type: type(Symbol) = RobogenSymbol):
        super().__init__(symbol_type,
                         self.generate_axiom,
                         self.generate_simple_rules)

    def generate_axiom(self, symbols, size: int):

        if size == 1:
            return self(size)

        axiom = SymbolicRepresentation(symbols)
        max_word_length = 3

        for i in range(max_word_length):

            axiom.append(self.symbol_type.BRACKET_STASH)
            orientations = self.symbol_type.orientation(symbols)
            if len(orientations) > 0:
                axiom.append(np.random.choice(orientations))
            axiom.append(np.random.choice(self.symbol_type.modules(symbols)))

            axiom.append(self.symbol_type.BRACKET_POP)

        return axiom

    def generate_rules(self, symbols, size: int):

        if size == 1:
            return self(size)

        replacement_rules = {}
        max_word_length = 4

        for symbol in self.symbol_type.symbols(symbols):
            # TODO probabilities
            replacement_rules[symbol] = SymbolicRepresentation(symbols)
            replacement_rules[symbol][:] = [random.choices(self.symbol_type.symbols(symbols), k=random.randint(1, max_word_length))]

        return replacement_rules

    def generate_simple_rules(self, symbols, size: int):

        if size == 1:
            return self(size)

        replacement_rules = {}
        max_word_length = 4

        for symbol in self.symbol_type.modules():
            replacement_rules[symbol] = SymbolicRepresentation(self.symbol_type)
            for i in range(random.randint(1, max_word_length)):
                # symbolic initialization
                replacement_rules[symbol].extend([random.choice(self.symbol_type.modules(symbols)),
                                                  random.choice(self.symbol_type.orientation(symbols))])  # probabilities

        return replacement_rules


class RobogenWordInitialization(RobogenInitialization):

    @classmethod
    def generate_rules(cls, size: int):
        replacement_rules = {}
        max_word_length = 5

        for word in RobogenWord.words():
            replacement_rules[word.module] = [[]]
            for i in range(random.randint(3, max_word_length)):
                replacement_rules[word.module][0].extend(cls.random_word())

        return replacement_rules

    @classmethod
    def random_word(cls, size: int, orientation=None, module=None, use_brackets: bool = None):
        sequence = []

        if use_brackets is None:
            use_brackets = random.randint(0, 1)

        if use_brackets:
            sequence.append(RobogenSymbol.BRACKET_STASH)

        if module is None:
            module = np.random.choice(RobogenSymbol.modules(), p=[0.5, 0.25, 0.25])
        if orientation is None:
            orientation = np.random.choice(RobogenSymbol.orientation(), p=[1 / 4, 1 / 4, 1 / 4, 1 / 4])

        sequence.extend(RobogenWord(module, orientation).symbols())

        if use_brackets:
            sequence.append(RobogenSymbol.BRACKET_POP)

        return sequence

    @classmethod
    def generate_axiom(cls, symbol, size: int):
        axiom = []
        for i in range(15):
            axiom.extend(cls.random_word(size, use_brackets=True))
        return axiom

