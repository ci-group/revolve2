from dataclasses import dataclass, field

from ._environment_controller import EnvironmentController
from ._posed_actor import PosedActor
from .geometry import Geometry


@dataclass
class Environment:
    """A list of posed actors."""

    controller: EnvironmentController
    actors: list[PosedActor] = field(default_factory=list, init=False)
    static_geometries: list[Geometry] = field(default_factory=list, init=False)
