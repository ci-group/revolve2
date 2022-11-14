"""Batch class."""

from dataclasses import dataclass, field
from typing import List

from ._environment import Environment


@dataclass
class Batch:
    """A set of environments and shared parameters for simulation."""

    simulation_time: int  # seconds

    """
    Hz. Frequency for state sampling during the simulation.
    The simulator will attempt to follow this as closely as possible,
    but is dependent on the actual step frequency of the simulator.
    """
    sampling_frequency: float

    """Similar to `sampling_frequency` but for how often the control function is called."""
    control_frequency: float

    """The environments to simulate."""
    environments: List[Environment] = field(default_factory=list, init=False)
