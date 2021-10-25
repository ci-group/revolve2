from dataclasses import dataclass, field
from typing import List

from .environment import Environment


@dataclass
class Batch:
    simulation_time: int  # seconds
    environments: List[Environment] = field(default_factory=list, init=False)
