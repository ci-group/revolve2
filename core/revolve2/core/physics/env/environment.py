from dataclasses import dataclass, field
from typing import Mapping

from ..actor import Actor


@dataclass
class Environment:
    actors: Mapping[str, Actor] = field(default_factory=dict, init=False)
