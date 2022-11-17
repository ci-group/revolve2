from dataclasses import dataclass, field
from typing import List

from ._environment_controller import EnvironmentController
from ._posed_actor import PosedActor

from dm_control.mjcf import RootElement

@dataclass
class Environment:
    """A list of posed actors."""

    controller: EnvironmentController
    actors: List[PosedActor] = field(default_factory=list, init=False)

    def amend (self, xml: RootElement):
        pass
