from __future__ import annotations

from dataclasses import dataclass
from typing import List, Set

from pyrr import Quaternion, Vector3


@dataclass
class ActorState:
    """State of an actor."""

    position: Vector3
    orientation: Quaternion

    # IDs of geometries (of this actor) in contact with ground
    groundcontacts: Set[int] = None
    # count of total geometries in Actor's morphology
    numgeoms: int = None


@dataclass
class EnvironmentState:
    """State of an environment."""

    time_seconds: float
    actor_states: List[ActorState]


@dataclass
class EnvironmentResults:
    """Result of running an environment."""

    environment_states: List[EnvironmentState]


@dataclass
class BatchResults:
    """Result of running a batch."""

    environment_results: List[EnvironmentResults]
