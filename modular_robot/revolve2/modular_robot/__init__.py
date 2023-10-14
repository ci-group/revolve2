"""Classes and functions to describe and work with modular robots as used in the CI Group at VU Amsterdam."""

from ._active_hinge import ActiveHinge
from ._body import Body
from ._body_state import BodyState
from ._brain import Brain
from ._brick import Brick
from ._core import Core
from ._create_batch_multiple_isolated_robots import (
    create_batch_multiple_isolated_robots,
)
from ._create_batch_single_robot import create_batch_single_robot
from ._get_body_states_multiple_isolated_robots import (
    get_body_states_multiple_isolated_robots,
)
from ._get_body_states_single_robot import get_body_states_single_robot
from ._modular_robot import ModularRobot
from ._module import Module
from ._morphological_measures import MorphologicalMeasures
from ._not_finalized_error import NotFinalizedError
from ._right_angles import RightAngles
from ._directions import Directions

__all__ = [
    "ActiveHinge",
    "Body",
    "BodyState",
    "Brain",
    "Brick",
    "Core",
    "ModularRobot",
    "Module",
    "MorphologicalMeasures",
    "NotFinalizedError",
    "RightAngles",
    "create_batch_multiple_isolated_robots",
    "create_batch_single_robot",
    "get_body_states_multiple_isolated_robots",
    "get_body_states_single_robot",
    "Directions",
]
