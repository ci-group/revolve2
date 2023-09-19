"""Classes and functions to describe and work with modular robots as used in the CI Group at VU Amsterdam."""

from revolve2.modular_robot._active_hinge import ActiveHinge
from revolve2.modular_robot._body import Body
from revolve2.modular_robot._body_state import BodyState
from revolve2.modular_robot._brain import Brain
from revolve2.modular_robot._brick import Brick
from revolve2.modular_robot._core import Core
from revolve2.modular_robot._create_batch_multiple_isolated_robots import (
    create_batch_multiple_isolated_robots,
)
from revolve2.modular_robot._create_batch_single_robot import create_batch_single_robot
from revolve2.modular_robot._get_body_states_multiple_isolated_robots import (
    get_body_states_multiple_isolated_robots,
)
from revolve2.modular_robot._get_body_states_single_robot import (
    get_body_states_single_robot,
)
from revolve2.modular_robot._modular_robot import ModularRobot
from revolve2.modular_robot._module import Module
from revolve2.modular_robot._not_finalized_error import NotFinalizedError
from revolve2.modular_robot._properties import Properties, PropertySet
from revolve2.modular_robot._right_angles import RightAngles
from revolve2.modular_robot.brains._morphological_measures import MorphologicalMeasures

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
    "Properties",
    "PropertySet",
    "RightAngles",
    "create_batch_multiple_isolated_robots",
    "create_batch_single_robot",
    "get_body_states_multiple_isolated_robots",
    "get_body_states_single_robot",
]
