import asyncio
import numpy as np
import json

import config
import revolve2.ci_group.rng
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group import terrains, fitness_functions
from revolve2.ci_group.rng import make_rng
from revolve2.ci_group.simulation import create_batch_single_robot_standard
from revolve2.modular_robot import ActiveHinge, Body, Brick, ModularRobot, RightAngles
from revolve2.modular_robot.brains import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot import ModularRobot, get_body_states_single_robot
from revolve2.simulators.mujoco import LocalRunner

from revolve2.experimentation.genotypes.cellular_automata.ca_genotype import CAGenotype
from revolve2.experimentation.genotypes.cellular_automata.develop import develop



def load_population_from_file(file_path):
    with open(file_path, 'r') as file:
        loaded_population = [json.loads(line) for line in file]
    
    # Convert keys from strings to tuples
    for individual in loaded_population:
        for key_str in list(individual.keys()):
            key_tuple = tuple(map(float, key_str.strip('()').split(', ')))
            value = individual.pop(key_str)
            individual[key_tuple] = value
    
    return loaded_population


def filter_unique_rules(population):
    unique_population = []
    seen_rules = set()

    for individual in population:
        # Convert the dictionary to a frozenset to make it hashable
        rule_set = frozenset(individual.items())

        # Check if the rule set is unique
        if rule_set not in seen_rules:
            unique_population.append(dict(rule_set))
            seen_rules.add(rule_set)

    return unique_population


file_path = 'examples/ca_experiments/last_population.json'
loaded_population = load_population_from_file(file_path)
unique_population = filter_unique_rules(loaded_population)

dsize = 10
domain = np.zeros((dsize, dsize))
domain[dsize // 2, dsize // 2] = 1

g = CAGenotype()
g.set_params(init_state=domain, iterations=6, rule_set=unique_population[0])
print(g.rule_set)
g.generate_body()
g.set_core(dsize // 2, dsize // 2)
print(g.ca_grid)

body = develop(g)

"""
body_measures = MorphologicalMeasures(body=body)
grid_arr = np.zeros_like(g.ca_grid)
grid = body_measures.body_as_grid
print("core in ca_grid", g.core_position)
print("core in body_measures", body_measures.core_grid_position)

for x in range(body_measures.bounding_box_depth):
    for y in range(body_measures.bounding_box_width):
        # if grid[x][y][body_measures.core_grid_position[2]] is not None:
        if "Brick" in str(grid[x][y][body_measures.core_grid_position[2]]):
            grid_arr[x][y] = 1.0
        elif "ActiveHinge" in str(grid[x][y][body_measures.core_grid_position[2]]):
            grid_arr[x][y] = 2.0
        elif "Core" in str(grid[x][y][body_measures.core_grid_position[2]]):
            grid_arr[x][y] = 3.0

print(grid)
"""

"""Run the simulation."""
# Set up a random number generater, used later.
RNG_SEED = 5
rng = make_rng(RNG_SEED)

# We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
brain = BrainCpgNetworkNeighborRandom(rng)
# Combine the body and brain into a modular robot.
robot = ModularRobot(body, brain)

# Create a batch containing the robot in a flat terrain.
# A batch describes a set of simulations to perform.
# In this case there is a single simulation containing a single robot.
# We use the 'standard' parameters for robot simulation.
# You probably don't have to worry about this, but if you are interested, take a look inside the function.

batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.flat())

runner = LocalRunner()
# Here we use the runner to run a batch.
# Once a runner finishes a batch, it can be reused to run a new batch.
# (We only run one batch in this tutorial, so we only use the runner once.)
asyncio.run(runner.run_batch(batch))