class BatchParameters:
    simulation_time: int | None
    sampling_frequency: float | None
    simulation_timestep: float
    control_frequency: float
    def __init__(self, *, simulation_time, sampling_frequency, simulation_timestep, control_frequency) -> None: ...
