"""Standard simulation functions and parameters patched for unit test."""
# This type-ignore is required, since the CI does not resolve those stubs correctly.
from revolve2.simulation.simulator import BatchParameters  # type: ignore


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
