from enum import auto

from nca.core.abstract.structural.tree.tree_helper import Orientation
from nca.core.genome.grammar.symbol import Symbol


def intersection(lst1, lst2):
    return [value for value in lst1 if value in lst2]


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
        return [cls.ORIENTATION_TOP, cls.ORIENTATION_RIGHT, cls.ORIENTATION_LEFT,
                cls.ORIENTATION_DOWN, cls.MODULE_BLOCK, cls.MODULE_HORIZONTAL_JOINT,
                cls.MODULE_VERTICAL_JOINT, cls.ORIENTATION_TOP, cls.ORIENTATION_DOWN,
                cls.ORIENTATION_LEFT, cls.ORIENTATION_RIGHT]

    @classmethod
    def actuators(cls):
        return [cls.MODULE_HORIZONTAL_JOINT, cls.MODULE_VERTICAL_JOINT]

    @classmethod
    def sensors(cls):
        return []

    @classmethod
    def orientation(cls, elements=None):
        possibilities = [cls.ORIENTATION_TOP, cls.ORIENTATION_RIGHT, cls.ORIENTATION_LEFT,
                    cls.ORIENTATION_DOWN]
        return possibilities if elements is None else intersection(elements, possibilities)

    @classmethod
    def modules(cls, elements=None):
        possibilities = [cls.MODULE_BLOCK, cls.MODULE_HORIZONTAL_JOINT, cls.MODULE_VERTICAL_JOINT]
        return possibilities if elements is None else intersection(elements, possibilities)

    @classmethod
    def brackets(cls, elements=None):
        return [cls.BRACKET_STASH, cls.BRACKET_POP]

    @classmethod
    def probabilities(cls, elements=None):
        return [0, 1/4, 1/8, 1/8, 1/8, 1/8, 1/8, 1/8, 0, 0]
