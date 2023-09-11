from enum import Enum, auto


class OpenMethod(Enum):
    """Describes the way a database should be opened."""

    OPEN_IF_EXISTS = auto()
    OPEN_OR_CREATE = auto()
    NOT_EXISTS_AND_CREATE = auto()
    OVERWITE_IF_EXISTS = auto()
