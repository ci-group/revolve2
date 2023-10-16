from __future__ import annotations

from enum import Enum, IntEnum, unique
from types import NotImplementedType
from typing import List, Literal
from math import inf, pi, isclose
import numbers


class RightAngles(Enum):
    """Standard angles at which  modular robot modules can be attached."""

    RAD_0 = float(0)
    RAD_HALFPI = pi / 2.0
    RAD_PI = pi
    RAD_ONEANDAHALFPI = pi / 2.0 * 3

    DEG_0 = 0
    DEG_90 = pi / 2.0
    DEG_180 = pi
    DEG_270 = pi / 2.0 * 3

    @staticmethod
    def circular(x: float) -> float:
        return x % (pi * 2.0)

    @classmethod
    def from_float(cls, value: float, strict: bool = False) -> CanonicalRightAngles:
        closest = min(cls, key=lambda x: abs(x.value - value))
        if strict and not isclose(closest.value, value):
            raise ValueError(f"{value} is not close to a right angle")
        return closest.as_canonical()

    def as_canonical(self) -> CanonicalRightAngles:
        match self:
            case RightAngles.DEG_0:
                return self.__class__.RAD_0
            case RightAngles.DEG_90:
                return self.__class__.RAD_HALFPI
            case RightAngles.DEG_180:
                return self.__class__.RAD_PI
            case RightAngles.DEG_270:
                return self.__class__.RAD_ONEANDAHALFPI
        return self

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, RightAngles):
            return isclose(self.value, __o.value)
        elif isinstance(__o, float):
            return isclose(self.value, __o)
        return NotImplemented

    def __add__(self, __o: object) -> CanonicalRightAngles | NotImplementedType:
        if isinstance(__o, RightAngles):
            return self.from_float(self.circular(self.value + __o.value), strict=True)
        elif isinstance(__o, numbers.Real):
            return self.from_float(self.circular(self.value + __o), strict=True)
        return NotImplemented

    def __sub__(self, __o: object) -> CanonicalRightAngles | NotImplementedType:
        if isinstance(__o, RightAngles):
            return self.from_float(self.circular(self.value - __o.value), strict=True)
        elif isinstance(__o, numbers.Real):
            return self.from_float(self.circular(self.value - __o), strict=True)
        return NotImplemented

    def __floordiv__(self, __o: object) -> CanonicalRightAngles | NotImplementedType:
        if isinstance(__o, numbers.Real):
            return self.from_float(self.circular(self.value // __o), strict=True)
        return NotImplemented

    def __mul__(self, __o: object) -> CanonicalRightAngles | NotImplementedType:
        if isinstance(__o, numbers.Real):
            return self.from_float(self.circular(self.value * __o), strict=True)
        return NotImplemented


CanonicalRightAngles = Literal[
    RightAngles.RAD_0,
    RightAngles.RAD_HALFPI,
    RightAngles.RAD_PI,
    RightAngles.RAD_ONEANDAHALFPI,
]


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
    def from_angle(cls, angle: RightAngles) -> Directions:
        match angle.as_canonical():
            case RightAngles.RAD_0:
                return cls.FRONT
            case RightAngles.RAD_HALFPI:
                return cls.RIGHT
            case RightAngles.RAD_PI:
                return cls.BACK
            case RightAngles.RAD_ONEANDAHALFPI:
                return cls.LEFT

    def to_angle(self) -> CanonicalRightAngles:
        match self:
            case self.FRONT:
                return RightAngles.RAD_0
            case self.LEFT:
                return RightAngles.RAD_ONEANDAHALFPI
            case self.RIGHT:
                return RightAngles.RAD_HALFPI
            case self.BACK:
                return RightAngles.RAD_PI
