from enum import Enum, auto


class Alphabet(Enum):

    @classmethod
    def list(cls):
        return list(map(lambda c: c, cls))


class TestColorAlphabet(Alphabet):
    RED = auto()
    GREEN = auto()
    BLUE = auto()
