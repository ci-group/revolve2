"""Standard simulation functions and parameters patched for unit test."""

from revolve2.simulation.simulator import BatchParameters


def make_patched_batch_parameters() -> BatchParameters:
    """
    Create batch parameters for unit-tests.

    :returns: The create batch parameters.
    """
    return BatchParameters(
        simulation_time=1,
        sampling_frequency=5,
        simulation_timestep=0.001,
        control_frequency=20,
    )
