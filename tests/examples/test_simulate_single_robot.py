import os

from ..conftest import EXAMPLES_DIR, assert_command_succeeds


def test_simulate_single_robot() -> None:
    """Test simulate_single_robot example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "simulate_single_robot")
    assert_command_succeeds(
        [
            "python",
            os.path.join(exp_dir, "main.py"),
            "-t",
            "1",
        ]
    )
