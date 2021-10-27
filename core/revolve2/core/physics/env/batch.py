from dataclasses import dataclass, field
from typing import Callable, Mapping

from .actor_control import ActorControl
from .environment import Environment


@dataclass
class Batch:
    simulation_time: int  # seconds
    control: Callable[[float, ActorControl], None]
    environments: Mapping[str, Environment] = field(default_factory=dict, init=False)
