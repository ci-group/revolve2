import glob
import os
import sys
import secrets

import pytest

# from dotenv import load_dotenv

TEST_DIR = os.path.abspath(os.path.dirname(__file__))
ROOT_DIR = os.path.abspath(os.path.dirname(TEST_DIR))

EXAMPLES_DIR = os.path.join(ROOT_DIR, "examples")


def pytest_sessionstart(session):
    """runs before all tests start https://stackoverflow.com/a/35394239"""
    print("in sessionstart")

    # here you could load ENV vars from a dotenv file for example
    # if not load_dotenv(override=True, dotenv_path=os.path.join(baseDir, '.env.test')):
    #    print('failed to load dotenv')
    #    exit(1)


@pytest.fixture(autouse=True)
def run_around_tests():
    """Code to run before and afer each test."""
    # code that will run before a given test:

    yield
    # code that will run after a given test:
    # for example here you could delete artifacts created by the test (e.g. sqlite files)


def get_uuid(length: int = 10):
    return str(secrets.token_hex(length))[:length]


class add_path:
    """
    Temporarily adds a given path to `sys.path`
    https://stackoverflow.com/a/39855753
    """

    def __init__(self, path: str):
        self.path = path

    def __enter__(self):
        sys.path.insert(0, self.path)

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            sys.path.remove(self.path)
        except ValueError as err:
            print(f"WARNING: failed to remove path from sys.path: '{self.path}'")
            # raise err
