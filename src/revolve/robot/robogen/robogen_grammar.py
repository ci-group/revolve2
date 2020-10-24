import random
from enum import auto

import numpy as np

from nca.core.abstract.structural.tree.tree_helper import Orientation
from nca.core.genome.grammar.grammar import Symbol, ReplacementRules


class RobogenSymbol(Symbol):

    MODULE_CORE = auto()
    MODULE_BLOCK = auto()

    MODULE_HORIZONTAL_JOINT = auto()
    MODULE_VERTICAL_JOINT = auto()

    ORIENTATION_TOP = Orientation.TOP
    ORIENTATION_RIGHT = Orientation.RIGHT
    ORIENTATION_LEFT = Orientation.LEFT
    ORIENTATION_DOWN = Orientation.DOWN

    BRACKET_STASH = auto()
    BRACKET_POP = auto()

    @classmethod
    def symbols(cls):
        return [RobogenSymbol.ORIENTATION_TOP, RobogenSymbol.ORIENTATION_RIGHT, RobogenSymbol.ORIENTATION_LEFT,
                RobogenSymbol.ORIENTATION_DOWN, RobogenSymbol.MODULE_BLOCK, RobogenSymbol.MODULE_HORIZONTAL_JOINT,
                RobogenSymbol.MODULE_VERTICAL_JOINT]

    @classmethod
    def joints(cls):
        return [RobogenSymbol.MODULE_HORIZONTAL_JOINT, RobogenSymbol.MODULE_VERTICAL_JOINT]

    @classmethod
    def orientation(cls):
        return [RobogenSymbol.ORIENTATION_TOP, RobogenSymbol.ORIENTATION_RIGHT, RobogenSymbol.ORIENTATION_LEFT,
                RobogenSymbol.ORIENTATION_DOWN]

    @classmethod
    def modules(cls):
        return [RobogenSymbol.MODULE_BLOCK, RobogenSymbol.MODULE_HORIZONTAL_JOINT, RobogenSymbol.MODULE_VERTICAL_JOINT]

    @classmethod
    def brackets(cls):
        return [RobogenSymbol.BRACKET_STASH, RobogenSymbol.BRACKET_POP]

    @classmethod
    def generate_axiom(self, max_word_length=3):
        axiom = []

        for i in range(max_word_length):

            axiom.append(RobogenSymbol.BRACKET_STASH)

            axiom.append(np.random.choice(RobogenSymbol.orientation()))
            axiom.append(np.random.choice(RobogenSymbol.modules(), p=[0.5, 0.25, 0.25]))

            axiom.append(RobogenSymbol.BRACKET_POP)

        return axiom

    @classmethod
    def generate_unconstrained_rules(self, max_word_length=4) -> ReplacementRules:
        replacement_rules: ReplacementRules = {}

        for symbol in RobogenSymbol.symbols():
            # TODO probabilities
            replacement_rules[symbol] = [random.choices(RobogenSymbol.symbols(), k=random.randint(1, max_word_length))]

        return replacement_rules

    @classmethod
    def generate_simple_rules(self, max_word_length=4) -> ReplacementRules:
        replacement_rules: ReplacementRules = {}

        for symbol in RobogenSymbol.modules():
            replacement_rules[symbol] = [[]]
            for i in range(random.randint(1, max_word_length)):
                replacement_rules[symbol][0].extend([random.choice(RobogenSymbol.modules()),
                                                  random.choice(RobogenSymbol.orientation())])  # probabilities

        return replacement_rules


class RobogenWord:

    def __init__(self, module: RobogenSymbol, orientation: RobogenSymbol = RobogenSymbol.ORIENTATION_TOP):
        self.module = module
        self.orientation = orientation

    def __hash__(self):
        return hash(hash(self.module) + hash(self.orientation))

    def __eq__(self, other):
        return self.module == other.module and self.orientation == other.orientation

    def __repr__(self):
        return self.module.name + " " + self.orientation.name

    def symbols(self):
        return [self.orientation, self.module]

    @classmethod
    def generate_rules(cls, max_word_length=3) -> ReplacementRules:
        replacement_rules: ReplacementRules = {}

        for word in RobogenWord.words():
            replacement_rules[word.module] = [[]]
            for i in range(random.randint(1, max_word_length)):
                replacement_rules[word.module][0].extend(cls.random_word())

        return replacement_rules

    @classmethod
    def random_word(cls, orientation=None, module=None, use_brackets: bool = None):
        sequence = []

        if use_brackets is None:
            use_brackets = random.randint(0, 1)

        if use_brackets:
            sequence.append(RobogenSymbol.BRACKET_STASH)

        if module is None:
           module = np.random.choice(RobogenSymbol.modules(), p=[0.5, 0.25, 0.25])
        if orientation is None:
            orientation = np.random.choice(RobogenSymbol.orientation(), p=[1/4, 1/4, 1/4, 1/4])

        sequence.extend(RobogenWord(module, orientation).symbols())

        if use_brackets:
            sequence.append(RobogenSymbol.BRACKET_POP)

        return sequence

    @classmethod
    def generate_axiom(self):
        axiom = []

        for orientation in RobogenSymbol.orientation():
            axiom.extend(RobogenWord.random_word(orientation, use_brackets=True))

        return axiom

    @classmethod
    def words(self):
        words = []
        for module in RobogenSymbol.modules():
            for orientation in RobogenSymbol.orientation():
                 words.append(RobogenWord(module, orientation))
        return words

"""
@classmethod
def random_root(cls, orientation=None, module=None):
    sequence = []

    sequence.append({'module': RobogenSymbol.BRACKET_STASH})

    if orientation is None:
        orientation = np.random.choice(RobogenSymbol.global_orientation(), p=[0.25, 0.25, 0.25, 0.25])
    if module is None:
        module = np.random.choice(RobogenSymbol.buildable_modules(), p=[1/2, 1/4, 1/4])
    sequence.append({'orientation': orientation, 'module': module})

    sequence.append({'module': RobogenSymbol.BRACKET_POP})

    return sequence
"""
