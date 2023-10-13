"""
This file configures pytest.
Then name `conftest.py` is pytest's default name and should not be changed.
"""

import os
import subprocess
import sys

import pytest

# from dotenv import load_dotenv

TEST_DIR = os.path.abspath(os.path.dirname(__file__))
ROOT_DIR = os.path.abspath(os.path.dirname(TEST_DIR))

EXAMPLES_DIR = os.path.join(ROOT_DIR, "examples")


def assert_command_succeeds(cmd: list):
    """Helper function for asserting a given command succeeds."""
    print("running command:\n", " ".join(cmd))
    res = subprocess.run(cmd, stdout=subprocess.PIPE)
    print(res.stdout)
    assert res.returncode == 0, f"expected returncode 0, got {res.returncode}"
