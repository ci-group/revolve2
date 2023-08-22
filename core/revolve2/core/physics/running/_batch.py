"""Batch class."""

from dataclasses import dataclass, field
from typing import List, Optional

from ._environment import Environment


@dataclass
class Batch:
    """A set of environments and shared parameters for simulation."""

    simulation_time: Optional[int]  # seconds
    """
    The duration for which each robot should be simulated.
    If set to 'None', the simulation will run indefinitely.
    """

    sampling_frequency: float
    """
    Hz. Frequency for state sampling during the simulation.
    The simulator will attempt to follow this as closely as possible,
    but is dependent on the actual step frequency of the simulator.
    """

    simulation_timestep: float
    """
    Simulation time step in seconds. This is an important parameter that affects the trade-off between speed and accuracy,
    as well as the stability of the robot model.
    Smaller values of this parameter generally result in better accuracy and stability.
    However, the specific value of this parameter may vary depending on the environments being simulated and other batch parameters.
    """

    control_frequency: float

    """Similar to `sampling_frequency` but for how often the control function is called."""

    environments: List[Environment] = field(default_factory=list, init=False)
    """The environments to simulate."""
