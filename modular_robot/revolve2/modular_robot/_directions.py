from enum import IntEnum, unique
from typing import List


@unique
class Directions(IntEnum):
    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    @classmethod
    def values(cls) -> List[int]:
        return [item.value for item in cls]

    @classmethod
    def has(cls, o: object, strict: bool = True) -> bool:
        if o is not None and o.__class__ == cls:
            return True
        return o in cls.values() and not strict
