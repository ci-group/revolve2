from enum import IntEnum, unique
from typing import List
from math import inf


@unique
class Directions(IntEnum):
    FRONT = 0
    LEFT = 1
    RIGHT = 2
    BACK = 3

    @classmethod
    def values(cls) -> List[int]:
        return [item.value for item in cls]

    @classmethod
    def has(cls, o: object, *, strict: bool = True, max_val: int | float = inf) -> bool:
        if o is not None and isinstance(o, cls):
            return o.value <= max_val
        return o in cls.values() and o <= max_val and not strict
