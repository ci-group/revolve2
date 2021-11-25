from dataclasses import dataclass
from typing import List

from pyrr import Quaternion, Vector3


@dataclass
class ActorState:
    position: Vector3
    orientation: Quaternion


@dataclass
class EnvironmentState:
    actor_states: List[ActorState]


@dataclass
class State:
    envs: List[EnvironmentState]
