from dataclasses import dataclass, field
from typing import List

from ._environment_controller import EnvironmentController
from ._posed_actor import PosedActor


@dataclass
class Environment:
    """A list of posed actors."""

    controller: EnvironmentController
    actors: List[PosedActor] = field(default_factory=list, init=False)
