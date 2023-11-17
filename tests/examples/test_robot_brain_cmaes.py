import os
import sys
from unittest.mock import Mock

from ..conftest import EXAMPLES_DIR
from ._clear_example_modules_from_cache import clear_example_modules_from_cache


def test_robot_brain_cmaes(mocker: Mock) -> None:
    """
    Test robot_brain_cmaes example can complete.

    :param mocker: The mock object.
    """
    example_dir = os.path.join(EXAMPLES_DIR, "robot_brain_cmaes")

    # Clear any previously imported modules from examples directory from cache
    clear_example_modules_from_cache()

    # Add examples directory to path, so we can import them without the examples being packages.
    sys.path.insert(0, example_dir)  # Add the example directory to sys.path

    # Override default config to reduce number of generations.
    mocker.patch("config.NUM_GENERATIONS", 2)

    # Import the example main and run it.
    try:
        import main  # type: ignore

        """This type ignore is required since mypy cant resolve this import."""

        main.main()
    finally:
        # Remove the example directory from path, even if the test fails.
        sys.path.pop(0)
