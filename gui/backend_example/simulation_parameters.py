"""Standard simulation functions and parameters."""

from revolve2.simulation.simulator import BatchParameters
from config_simulation_parameters import SAMPLING_FREQUENCY, SIMULATION_TIMESTEP, CONTROL_FREQUENCY
from config import SIMULATION_TIME

def make_batch_parameters(
    simulation_time: int = SIMULATION_TIME,
    sampling_frequency: float | None = SAMPLING_FREQUENCY,
    simulation_timestep: float = SIMULATION_TIMESTEP,
    control_frequency: float = CONTROL_FREQUENCY,
) -> BatchParameters:
    """
    Create batch parameters as standardized within the CI Group.

    :param simulation_time: As defined in the `BatchParameters` class.
    :param sampling_frequency: As defined in the `BatchParameters` class.
    :param simulation_timestep: As defined in the `BatchParameters` class.
    :param control_frequency: As defined in the `BatchParameters` class.
    :returns: The create batch parameters.
    """
    return BatchParameters(
        simulation_time=simulation_time,
        sampling_frequency=sampling_frequency,
        simulation_timestep=simulation_timestep,
        control_frequency=control_frequency,
    )
