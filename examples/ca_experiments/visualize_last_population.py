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
from revolve2.modular_robot.representations import render_robot



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

for i in range(len(unique_population)):
    g = CAGenotype()
    g.set_params(init_state=domain, iterations=6, rule_set=unique_population[i])
    print(g.rule_set)
    g.generate_body()
    g.set_core(dsize // 2, dsize // 2)

    body = develop(g)

    RNG_SEED = 5
    rng = make_rng(RNG_SEED)
    # # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
    brain = BrainCpgNetworkNeighborRandom(rng)
    # # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)


    bodyplan = render_robot(robot, f'/home/user/Desktop/revolve2-adv-symmetry-main/examples/ca_experiments/bodyplan{i}.png')



# """Run the simulation."""
# # Set up a random number generater, used later.
# RNG_SEED = 5
# rng = make_rng(RNG_SEED)

# # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
# brain = BrainCpgNetworkNeighborRandom(rng)
# # Combine the body and brain into a modular robot.
# robot = ModularRobot(body, brain)

# # Create a batch containing the robot in a flat terrain.
# # A batch describes a set of simulations to perform.
# # In this case there is a single simulation containing a single robot.
# # We use the 'standard' parameters for robot simulation.
# # You probably don't have to worry about this, but if you are interested, take a look inside the function.

# batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.flat())

# runner = LocalRunner()
# # Here we use the runner to run a batch.
# # Once a runner finishes a batch, it can be reused to run a new batch.
# # (We only run one batch in this tutorial, so we only use the runner once.)
# asyncio.run(runner.run_batch(batch))