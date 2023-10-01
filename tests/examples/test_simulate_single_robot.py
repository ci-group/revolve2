import glob
import os
import subprocess

from tests.conftest import EXAMPLES_DIR

EXPERIMENT_DIR = os.path.join(EXAMPLES_DIR, "simulate_single_robot")


EXP_CMD_BASE = [
    "python3",
    os.path.join(EXPERIMENT_DIR, "main.py"),
    "-t",
    "2",
]


def test_experiment_can_complete():
    """Test that main.py can complete (without crashing)."""
    cmd = EXP_CMD_BASE.copy()
    print("running command:")
    print(cmd)
    print(" ".join(cmd))
    res = subprocess.run(cmd, stdout=subprocess.PIPE)

    print(res.stdout)
    assert res.returncode == 0, f"expected returncode 0, got {res.returncode}"
