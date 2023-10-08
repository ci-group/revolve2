import os

from tests.conftest import EXAMPLES_DIR, assert_command_succeeds


def test_evaluate_multiple_isolated_robots():
    """Test evaluate_multiple_isolated_robots example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "evaluate_multiple_isolated_robots")
    assert_command_succeeds(["python3", os.path.join(exp_dir, "main.py")])


def test_evaluate_single_robot():
    """Test evaluate_single_robot example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "evaluate_single_robot")
    assert_command_succeeds(["python3", os.path.join(exp_dir, "main.py"), "-t", "1"])


def test_experiment_setup():
    """Test experiment_setup example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "experiment_setup")
    assert_command_succeeds(["python3", os.path.join(exp_dir, "main.py")])


def test_simple_ea_xor():
    """Test simple_ea_xor example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "simple_ea_xor")
    assert_command_succeeds(["python3", os.path.join(exp_dir, "main.py")])


def test_simulate_single_robot():
    """Test simulate_single_robot example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "simulate_single_robot")
    assert_command_succeeds(
        [
            "python3",
            os.path.join(exp_dir, "main.py"),
            "-t",
            "1",
        ]
    )
