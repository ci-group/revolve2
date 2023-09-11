"""Configuration parameters for this example."""

from revolve2.standard_resources.modular_robots import gecko

DATABASE_FILE = "database.sqlite"
NUM_REPETITIONS = 5
NUM_SIMULATORS = 8
INITIAL_STD = 0.5
NUM_GENERATIONS = 100
BODY = gecko()
