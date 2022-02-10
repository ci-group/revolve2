from dataclasses import dataclass, field
from typing import List

from ._posed_actor import PosedActor


@dataclass
class Environment:
    actors: List[PosedActor] = field(default_factory=list, init=False)
