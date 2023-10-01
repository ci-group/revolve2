import os
import subprocess
import sys
import pytest
from tests.conftest import EXAMPLES_DIR, add_path
from unittest.mock import patch

EXP_DIR = os.path.join(EXAMPLES_DIR, "robot_brain_cmaes")

EXP_CMD_BASE = [
    "python3",
    os.path.join(EXP_DIR, "main.py"),
]


def mock_config():
    print("in mock_config")
    import examples.robot_brain_cmaes.config as config

    overrides = {
        "NUM_SIMULATORS": 4,
        # "INITIAL_STD": 0.5,
        "NUM_GENERATIONS": 2,
    }
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


# disabled for now because this test fails in the CI for some reason
#   TypeError: Evaluator.evaluate() takes 2 positional arguments but 5 were given
#   ^ I think it's maybe importing the evaluator from another example directory?


@pytest.mark.cwd(EXP_DIR)
# @patch("examples.robot_brain_cmaes.main.get_config", mock_config())
# @patch("config", mock_config())
def test_experiment_can_complete(custom_cwd, mocker):
    """Test that main.py can complete (without crashing)."""
    print("\nFLAG CWD:", os.getcwd())
    # from examples.robot_brain_cmaes.main import main
    # import main as main_script
    mocker.patch(
        "examples.robot_brain_cmaes.main.get_config", return_value=mock_config()
    )
    # mocker.patch("robot_brain_cmaes.main.get_config", return_value=mock_config())
    # mocker.patch("main_script.get_config", return_value=mock_config())

    """
    cmd = EXP_CMD_BASE.copy()
    print("running command:")
    print(cmd)
    print(" ".join(cmd))
    res = subprocess.run(cmd, stdout=subprocess.PIPE, cwd=EXP_DIR)

    print(res.stdout)
    assert res.returncode == 0, f"expected returncode 0, got {res.returncode}"
    """

    import examples.robot_brain_cmaes.main as main_script

    # with add_path(os.path.join(EXAMPLES_DIR, "robot_brain_cmaes")):
    print("importing main")
    print()
    main_script.main()
