"""Batch class."""

from dataclasses import dataclass, field

from ..scene import Scene
from ._batch_parameters import BatchParameters
from ._record_settings import RecordSettings


@dataclass
class Batch:
    """A set of scenes and shared parameters for simulation."""

    parameters: BatchParameters

    scenes: list[Scene] = field(default_factory=list, init=False)
    """The scenes to simulate."""

    record_settings: RecordSettings | None = None
