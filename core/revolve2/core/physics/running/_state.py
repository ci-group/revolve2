"""
State class and subclasses used by it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

from pyrr import Quaternion, Vector3


@dataclass
class ActorState:
    """
    State of an actor.
    """

    position: Vector3
    orientation: Quaternion


@dataclass
class EnvironmentState:
    """
    State of an environment.
    """

    actor_states: List[ActorState]


@dataclass
class RunnerState:
    """
    State of a runner.
    """

    time_seconds: float
    envs: List[EnvironmentState]
