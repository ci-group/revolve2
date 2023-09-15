import math
from enum import Enum


class RightAngles(Enum):
    """Standard angles at which  modular robot modules can be attached."""

    DEG_0 = 0
    DEG_90 = math.pi / 2.0
    DEG_180 = math.pi
    DEG_270 = math.pi / 2.0 * 3
    RAD_0 = 0
    RAD_HALFPI = math.pi / 2.0
    RAD_PI = math.pi
    RAD_ONEANDAHALFPI = math.pi / 2.0 * 3
