from __future__ import annotations

from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class ActorState:
    """State of an actor."""

    position: Vector3
    orientation: Quaternion


@dataclass
class EnvironmentState:
    """State of an environment."""

    time_seconds: float
    actor_states: list[ActorState]


@dataclass
class EnvironmentResults:
    """Result of running an environment."""

    environment_states: list[EnvironmentState]


@dataclass
class BatchResults:
    """Result of running a batch."""

    environment_results: list[EnvironmentResults]
