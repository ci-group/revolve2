"""Unit tests for examples."""
from ._test_evaluate_multiple_isolated_robots import (
    test_evaluate_multiple_isolated_robots,
)
from ._test_evaluate_single_robot import test_evaluate_single_robot
from ._test_experiment_setup import test_experiment_setup
from ._test_robot_bodybrain_ea_database import test_robot_bodybrain_ea_database
from ._test_robot_brain_cmaes import test_robot_brain_cmaes
from ._test_simple_ea_xor import test_simple_ea_xor
from ._test_simulate_single_robot import test_simulate_single_robot

__all__ = [
    "test_evaluate_multiple_isolated_robots",
    "test_evaluate_single_robot",
    "test_experiment_setup",
    "test_robot_bodybrain_ea_database",
    "test_robot_brain_cmaes",
    "test_simple_ea_xor",
    "test_simulate_single_robot",
]
