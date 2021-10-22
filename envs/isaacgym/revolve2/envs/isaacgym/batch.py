from dataclasses import dataclass, field
from typing import List

from .environment import Environment


@dataclass
class Batch:
    environments: List[Environment] = field(default_factory=list, init=False)
