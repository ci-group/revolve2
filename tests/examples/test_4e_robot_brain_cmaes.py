import os
import sys
from unittest.mock import Mock

from ..conftest import EXAMPLES_DIR
from ._clear_example_modules_from_cache import clear_exp_modules_from_cache


def test_4e_robot_brain_cmaes(mocker: Mock) -> None:
    """
    Test 4e_robot_brain_cmaes example can complete.

    :param mocker: The mock object.
    """
    exp_dir = os.path.join(
        EXAMPLES_DIR, "4_example_experiment_setups/4e_robot_brain_cmaes"
    )

    # Clear any previously imported modules from examples directory from cache
    clear_exp_modules_from_cache()

    # Add examples directory to path, so we can import them without the examples being packages.
    sys.path.insert(0, exp_dir)  # Add the example directory to sys.path

    # Override default config to reduce number of generations.
    mocker.patch("config.NUM_GENERATIONS", 2)

    # Import the example main and run it.
    try:
        # This type ignore is required since mypy cant resolve this import.
        import main  # type: ignore

        main.main()
    finally:
        # Remove the example directory from path, even if the test fails.
        sys.path.pop(0)
