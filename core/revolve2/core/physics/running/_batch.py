from dataclasses import dataclass, field
from typing import Callable, List

from ._actor_control import ActorControl
from ._environment import Environment


@dataclass
class Batch:
    simulation_time: int  # seconds

    """
    Hz. Frequency for state sampling during the simulation.
    The simulator will attempt to follow this as closely as possible,
    but is dependent on the actual step frequency of the simulator.
    """
    sampling_frequency: float
    control_frequency: float  # Hz. See `sampling_frequency`, but for actor control.
    control: Callable[
        [int, float, ActorControl], None
    ]  # (environment_index, dt, control) -> None
    environments: List[Environment] = field(default_factory=list, init=False)
