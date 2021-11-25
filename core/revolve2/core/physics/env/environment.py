from dataclasses import dataclass, field
from typing import List

from ..actor import Actor


@dataclass
class Environment:
    actors: List[Actor] = field(default_factory=list, init=False)
