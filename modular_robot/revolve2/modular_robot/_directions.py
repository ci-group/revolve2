from __future__ import annotations

from enum import Enum, IntEnum, unique
from typing import List
from math import inf, pi, isclose
import numbers


class RightAngles(Enum):
    """Standard angles at which  modular robot modules can be attached."""

    RAD_0 = float(0)
    RAD_HALFPI = pi / 2.0
    RAD_PI = pi
    RAD_ONEANDAHALFPI = pi / 2.0 * 3

    @classmethod
    def from_float(cls, value: float) -> RightAngles:
        return min(cls, key=lambda x: abs(x.value - value))

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, RightAngles):
            return isclose(self.value, __o.value)
        elif isinstance(__o, float):
            return self == RightAngles.from_float(__o)
        return NotImplemented

    def __add__(self, __o: object) -> RightAngles:
        if isinstance(__o, RightAngles):
            return self.from_float((self.value + __o.value) % (pi * 2))
        elif isinstance(__o, numbers.Real):
            return self.from_float((self.value + __o) % (pi * 2))
        return NotImplemented

    def __sub__(self, __o: object) -> RightAngles:
        if isinstance(__o, RightAngles):
            return self.from_float((self.value - __o.value) % (pi * 2))
        elif isinstance(__o, numbers.Real):
            return self.from_float((self.value - __o) % (pi * 2))
        return NotImplemented

    def __floordiv__(self, __o: object) -> RightAngles:
        if isinstance(__o, numbers.Real):
            return self.from_float((self.value // __o) % (pi * 2))
        return NotImplemented

    def __mul__(self, __o: object) -> RightAngles:
        if isinstance(__o, numbers.Real):
            return self.from_float((self.value * __o) % (pi * 2))
        return NotImplemented


@unique
class Directions(IntEnum):
    """The directions a module can have children in.
    Used for indexing the children array, and coupled with that.
    """

    # This order is needed to allow the hinge to only have front
    # ... and the block to not have back
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

    @classmethod
    def from_angle(cls, angle: RightAngles):
        match angle:
            case RightAngles.RAD_0:
                return cls.FRONT
            case RightAngles.RAD_HALFPI:
                return cls.RIGHT
            case RightAngles.RAD_PI:
                return cls.BACK
            case RightAngles.RAD_ONEANDAHALFPI:
                return cls.LEFT

    def to_angle(self) -> RightAngles:
        match self:
            case self.FRONT:
                return RightAngles.RAD_0
            case self.LEFT:
                return RightAngles.RAD_ONEANDAHALFPI
            case self.RIGHT:
                return RightAngles.RAD_HALFPI
            case self.BACK:
                return RightAngles.RAD_PI
