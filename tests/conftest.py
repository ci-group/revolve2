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


@pytest.fixture
def custom_cwd(request):
    """Fixture to set a custom current working directory (and update sys.path) for a single test."""
    # Get the desired custom cwd from the test function
    desired_cwd_obj = request.node.get_closest_marker("cwd")

    if desired_cwd_obj:
        desired_cwd = desired_cwd_obj.args[0]
        # Save the current working directory
        original_cwd = os.getcwd()

        # Change the current working directory to the desired custom cwd
        os.chdir(desired_cwd)

        print(f"cwd changed to '{os.getcwd()}', starting test...")
        sys.path.insert(0, desired_cwd)
        yield  # Execute the test

        # Restore the original working directory when the test is done
        os.chdir(original_cwd)
        print(f"restored cwd to '{os.getcwd()}'")
        sys.path.remove(desired_cwd)
        assert os.getcwd() == original_cwd
    else:
        yield  # No custom cwd specified, just execute the test


# TODO: delete this?
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
