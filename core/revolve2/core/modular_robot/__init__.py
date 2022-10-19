"""Classes and functions to describe and work with modular robots as used in the CI Group at VU Amsterdam."""

from ._active_hinge import ActiveHinge
from ._body import Body
from ._brain import Brain
from ._brick import Brick
from ._core import Core
from ._modular_robot import ModularRobot
from ._module import Module
from ._morphological_measures import MorphologicalMeasures
from ._not_finalized_error import NotFinalizedError

__all__ = [
    "ActiveHinge",
    "Body",
    "Brain",
    "Brick",
    "Core",
    "ModularRobot",
    "Module",
    "MorphologicalMeasures",
    "NotFinalizedError",
]
