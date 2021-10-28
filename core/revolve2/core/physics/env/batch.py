from dataclasses import dataclass, field
from typing import Callable, List

from .actor_control import ActorControl
from .environment import Environment


@dataclass
class Batch:
    simulation_time: int  # seconds
    control: Callable[[float, ActorControl], None]  # (dt, control) -> None
    environments: List[Environment] = field(default_factory=list, init=False)
