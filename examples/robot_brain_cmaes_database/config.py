"""Configuration parameters for this example."""

from revolve2.ci_group.modular_robots import gecko

DATABASE_FILE = "database.sqlite"
NUM_REPETITIONS = 5
NUM_SIMULATORS = 8
INITIAL_STD = 0.5
NUM_GENERATIONS = 2
BODY = gecko()
