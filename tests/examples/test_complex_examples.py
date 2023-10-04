"""Tests some of the more "complex" examples that require mocking or a temporary directory for artifacts."""

import os
from types import ModuleType


def mock_config(config: ModuleType, overrides: dict):
    """Helper function for overriding desired items in a config object."""
    for key, value in overrides.items():
        # sanity check
        assert (
            getattr(config, key, None) is not None
        ), f"key '{key}' not found in config"
        setattr(config, key, value)
    return config


def test_robot_bodybrain_cmaes(mocker):
    """Test robot_bodybrain_cmaes example can complete."""
    import examples.robot_brain_cmaes.config as config

    overrides = {
        "NUM_GENERATIONS": 2,
    }
    # override default config (to reduce number of generations)
    mocker.patch(
        "examples.robot_brain_cmaes.main.get_config",
        return_value=mock_config(config, overrides),
    )
    import examples.robot_brain_cmaes.main as main_script

    main_script.main()


def test_robot_bodybrain_ea(mocker):
    """Test robot_bodybrain-ea example can complete."""
    import examples.robot_bodybrain_ea.config as config

    # override default config (to reduce number of generations)
    overrides = {
        "POPULATION_SIZE": 4,
        "OFFSPRING_SIZE": 2,
        "NUM_GENERATIONS": 2,
    }
    mocker.patch(
        "examples.robot_bodybrain_ea.main.get_config",
        return_value=mock_config(config, overrides),
    )
    import examples.robot_bodybrain_ea.main as main_script

    main_script.main()


def test_robot_bodybrain_ea_database(mocker, tmpdir):
    """
    Test robot_bodybrain_ea_database example can complete.
    Database file is written to a temporary directory automatically deleted after the test.
    """
    import examples.robot_bodybrain_ea_database.config as config

    # override default config (to reduce number of generations)
    overrides = {
        "DATABASE_FILE": os.path.join(tmpdir, "database.sqlite"),
        "NUM_REPETITIONS": 1,
        "POPULATION_SIZE": 4,
        "OFFSPRING_SIZE": 2,
        "NUM_GENERATIONS": 2,
    }

    mocker.patch(
        "examples.robot_bodybrain_ea_database.main.get_config",
        return_value=mock_config(config, overrides),
    )
    import examples.robot_bodybrain_ea_database.main as main_script

    main_script.main()
    assert os.path.exists(overrides["DATABASE_FILE"])
    # here we could also connect to the database and make assertions about its state
