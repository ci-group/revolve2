from enum import Enum, auto


class OpenCheck(Enum):
    OPEN_IF_EXISTS = auto()
    OPEN_OR_CREATE = auto()
    NOT_EXISTS_AND_CREATE = auto()
    OVERWITE_IF_EXISTS = auto()
