import os
import sys
from unittest.mock import Mock

from ..conftest import EXAMPLES_DIR
from ._clear_example_modules_from_cache import clear_exp_modules_from_cache


def test_robot_bodybrain_ea_database(mocker: Mock, tmpdir: str) -> None:
    """
    Test if 4d_robot_bodybrain_ea_database example can complete.

    The database file is written to a temporary directory, which is automatically deleted after the test.

    :param mocker: The mock object.
    :param tmpdir: Temporary directory for spawning database file.
    """
    exp_dir = os.path.join(
        EXAMPLES_DIR, "4_example_experiment_setups/4d_robot_bodybrain_ea_database"
    )

    # Clear any previously imported modules from examples directory from cache
    clear_exp_modules_from_cache()

    # Add examples directory to path, so we can import them without the examples being packages.
    sys.path.insert(0, exp_dir)

    # Override default config to reduce number of generations.
    database_file = os.path.join(tmpdir, "database.sqlite")
    mocker.patch("config.DATABASE_FILE", database_file)
    mocker.patch("config.NUM_REPETITIONS", 1)
    mocker.patch("config.POPULATION_SIZE", 4)
    mocker.patch("config.OFFSPRING_SIZE", 2)
    mocker.patch("config.NUM_GENERATIONS", 2)

    # Import the example main and run it.
    try:
        # This type ignore is required since mypy cant resolve this import."
        import main  # type: ignore

        main.main()
    finally:
        # Remove the example directory from path, even if the test fails.
        sys.path.pop(0)

    # Confirm that database was created.
    assert os.path.exists(database_file)
