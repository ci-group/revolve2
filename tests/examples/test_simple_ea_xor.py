import os

from ..conftest import EXAMPLES_DIR, assert_command_succeeds


def test_simple_ea_xor() -> None:
    """Test 4a_simple_ea_xor example can complete."""
    exp_dir = os.path.join(EXAMPLES_DIR, "4_example_experiment_setups/4a_simple_ea_xor")
    assert_command_succeeds(["python", os.path.join(exp_dir, "main.py")])
