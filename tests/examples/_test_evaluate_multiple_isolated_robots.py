"""Tests some of the examples that can just be easily ran as a subprocess."""
import os

from ..conftest import EXAMPLES_DIR, assert_command_succeeds


def test_evaluate_multiple_isolated_robots() -> None:
    """Test evaluate_multiple_isolated_robots example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "evaluate_multiple_isolated_robots")
    assert_command_succeeds(["python3", os.path.join(exp_dir, "main.py")])
