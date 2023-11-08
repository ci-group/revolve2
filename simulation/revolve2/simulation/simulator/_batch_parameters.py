from dataclasses import dataclass


@dataclass(kw_only=True)
class BatchParameters:
    """Parameters for a simulation batch."""

    simulation_time: int | None
    """
    Seconds. The duration for which each robot should be simulated.
    If set to 'None', the simulation will run indefinitely.
    """

    sampling_frequency: float | None
    """
    Hz. Frequency for state sampling during the simulation.
    The simulator will attempt to follow this as closely as possible,
    but is dependent on the actual step frequency of the simulator.
    If set to 'None', no sampling will be performed.
    """

    simulation_timestep: float
    """
    Simulation time step in seconds. This is an important parameter that affects the trade-off between speed and accuracy,
    as well as the stability of the robot model.
    Smaller values of this parameter generally result in better accuracy and stability.
    However, the specific value of this parameter may vary depending on the scenes being simulated and other batch parameters.
    """

    control_frequency: float
    """Similar to `sampling_frequency` but for how often the control function is called."""
