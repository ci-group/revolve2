"""
This file configures pytest.

The name `conftest.py` is pytest's default name and should not be changed.
"""

import os
import subprocess

TEST_DIR = os.path.abspath(os.path.dirname(__file__))
ROOT_DIR = os.path.abspath(os.path.dirname(TEST_DIR))

EXAMPLES_DIR = os.path.join(ROOT_DIR, "examples")


def assert_command_succeeds(cmd: list[str]) -> None:
    """
    Assert if a given command succeeds.

    :param cmd: Slices of the command.
    """
    print("running command:\n", " ".join(cmd))
    res = subprocess.run(cmd, stdout=subprocess.PIPE)
    print(res.stdout)
    assert res.returncode == 0, f"expected return code 0, got {res.returncode}"
