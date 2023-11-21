import os
import sys
from unittest.mock import Mock

from ..conftest import EXAMPLES_DIR
from ._clear_example_modules_from_cache import clear_exp_modules_from_cache
from ._patched_batch_parameters import make_patched_batch_parameters


def test_evaluate_single_robot(mocker: Mock) -> None:
    """
    Test evaluate_single_robot example can complete.

    :param mocker: The mock object.
    """
    exp_dir = os.path.join(EXAMPLES_DIR, "evaluate_single_robot")
    # Clear any previously imported modules from examples directory from cache
    clear_exp_modules_from_cache()
    # Add examples directory to path, so we can import them without the examples being packages.
    sys.path.insert(0, exp_dir)
    # Override import for patching batch parameters.
    mocker.patch("main.make_standard_batch_parameters", make_patched_batch_parameters)
    try:
        # This type ignore is required since mypy cant resolve this import.
        import main  # type: ignore

        main.main()
    finally:
        # Remove the example directory from path, even if the test fails.
        sys.path.pop(0)
