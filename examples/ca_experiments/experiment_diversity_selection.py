"""
CA representation testing, generates robot with simple diversity selection with Jaccard similarity.
Starting gene length (dict of CA rules) is fixed.
Mutation replaces one random chosen rule with a randomly 
generated rule 

Run this file to start experiment. Visualize population by running visualize_last_population.py
"""

import logging
import numpy as np
import asyncio
import json
import matplotlib.pyplot as plt

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
from revolve2.modular_robot import MorphologicalMeasures

from revolve2.experimentation.genotypes.cellular_automata.ca_genotype import CAGenotype
from revolve2.experimentation.genotypes.cellular_automata.develop import develop


def initialize():
    population = []

    for i in range(config.NUM_INDIVIDUALS):
        rng = revolve2.ci_group.rng.make_rng_time_seed()

        # If you run with a set seed, use the following lines instead.
        # SEED = 1234
        # rng = revolve2.ci_group.rng.make_rng(SEED)

        dsize = 10
        domain = np.zeros((dsize, dsize))
        domain[dsize // 2, dsize // 2] = 1

        g = CAGenotype()
        g.set_params(init_state=domain, iterations=6, rule_set={})
        g.generate_random_genotype(10)
        g.generate_body()
        g.set_core(dsize // 2, dsize // 2)

        body = develop(g)

        population.append(g.rule_set)

    return population, domain


def run_experiment(previous_population, init_domain):
    """
    Run all runs of an experiment using the provided parameters.

    """
    # Create a list where we will store the success ratio for each repetition.

    generation_fitness = []
    current_population = []

    for i in range(len(previous_population)):
        rng = revolve2.ci_group.rng.make_rng_time_seed()

        g = CAGenotype()

        g.rule_set = previous_population[i]
        g.init_state = init_domain.copy()
        g.mutate()
        g.generate_body()
        dsize = len(g.ca_grid)
        g.set_core(dsize // 2, dsize // 2)

        body = develop(g)

        # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
        brain = BrainCpgNetworkNeighborRandom(rng)
        # Combine the body and brain into a modular robot.
        robot = ModularRobot(body, brain)

        batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.flat())

        runner = LocalRunner(headless=True)

        results = asyncio.run(runner.run_batch(batch))
        environment_results = results.environment_results[0]

        # We have to map the simulation results back to robot body space.
        # This function calculates the state of the robot body at the start and end of the simulation.
        body_state_begin, body_state_end = get_body_states_single_robot(
            body, environment_results
        )

        # Calculate the xy displacement from the body states.
        xy_displacement = fitness_functions.xy_displacement(
            body_state_begin, body_state_end
        )

        generation_fitness.append(xy_displacement)
        logging.info(f"xy_displacement = {xy_displacement}")

        current_population.append(g.rule_set)

    return generation_fitness, current_population

def body_to_num_grid(body):
    """
    
    """
    body_measures = MorphologicalMeasures(body)
    revolve_grid = body_measures.body_as_grid

    grid = np.zeros((body_measures.bounding_box_depth, body_measures.bounding_box_width))

    for x in range(body_measures.bounding_box_depth):
        for y in range(body_measures.bounding_box_width):
            # if grid[x][y][body_measures.core_grid_position[2]] is not None:
            if "Brick" in str(revolve_grid[x][y][body_measures.core_grid_position[2]]):
                grid[x][y] = 1
            elif "ActiveHinge" in str(revolve_grid[x][y][body_measures.core_grid_position[2]]):
                grid[x][y] = 2
            elif "Core" in str(revolve_grid[x][y][body_measures.core_grid_position[2]]):
                grid[x][y] = 3

    return grid

def convert_grid_to_set(grid):
    number_positions = {0: set(), 1: set(), 2: set(), 3: set()}
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            number_positions[grid[i][j]].add((i, j))
    return number_positions

# Diversity measure
def jaccard_similarity(grid1, grid2):
    set1 = convert_grid_to_set(grid1)
    set2 = convert_grid_to_set(grid2)
    
    intersection = sum(len(set1[num] & set2[num]) for num in range(4))
    union = sum(len(set1[num] | set2[num]) for num in range(4))
    
    similarity = intersection / union if union != 0 else 0  # Avoid division by zero
    
    return similarity


def survivor_selection_diversity(population, iters=6, similarity_cutoff=0.5, gene_length=10):
    """
    Pairwise selection between all individuals of a population.

    If similarity(individ_i, individ_j) > similarity_cutoff => discard individ_j
    Else pass individ_j to next population

    If individual is deleted, replaces it with a random individual (i.e. with a random rule set)

    param iters:  number of CA iterations for generating body 
    """
    next_population = population.copy()

    dsize = 10
    init_domain = np.zeros((dsize, dsize))
    init_domain[dsize // 2, dsize // 2] = 1

    for i in range(len(next_population) - 2):
        for j in range(len(next_population) - 2):
            print(f'i, j = {i, j}')
            if i != j:
                g1 = CAGenotype()
                g1.rule_set = population[i]
                g1.init_state = init_domain.copy()
                g1.generate_body()
                g1.set_core(dsize // 2, dsize // 2)
                body1 = develop(g1)
                grid1 = body_to_num_grid(body=body1)

                g2 = CAGenotype()
                g2.rule_set = population[j]
                g2.init_state = init_domain.copy()
                g2.generate_body()
                g2.set_core(dsize // 2, dsize // 2)
                body2 = develop(g2)
                grid2 = body_to_num_grid(body=body2)

                if jaccard_similarity(grid1, grid2) > similarity_cutoff:
                    del next_population[j]

                # if a rule set was deleted, add random rule set to population
                if len(population) > len(next_population):
                    while len(population) > len(next_population):
                        g = CAGenotype()
                        g.set_params(init_state=init_domain, iterations=iters, rule_set={})
                        g.generate_random_genotype(gene_length)
                        next_population.append(g.rule_set)

    return next_population



def save_population_to_file(population, file_path):
    with open(file_path, 'w') as file:
        for individual in population: 
            converted_individual = {str(k): v for k, v in individual.items()}
            json.dump(converted_individual, file)
            file.write('\n')



def main() -> None:
    """Run the simulation."""
    # Set up standard logging.
    # This decides the level of severity of logged messages we want to display.
    # By default this is 'INFO' or more severe, and 'DEBUG' is excluded.
    # Furthermore, a standard message layout is set up.
    # If logging is not set up, important messages can be missed.
    # We also provide a file name, so the log will be written to both the console and that file.
    setup_logging(file_name="log.txt")

    # Let's print a simple message.
    # We use the 'info' function to give the message the 'INFO' severity.
    logging.info("Starting program.")
    # The following message will be invisible.
    logging.debug(
        "This debug message is invisible, because we set our visible severity to 'INFO' and higher."
    )

    fitness_data = []
    population, init_domain = initialize()
    for repetition in range(config.NUM_GENERATIONS):
        generation_fitness, population_next = run_experiment(population, init_domain)
        fitness_data.append([generation_fitness])
        population = survivor_selection_diversity(population_next.copy())

    plt.figure()
    plt.title(f'# generations = {config.NUM_GENERATIONS}, population size = {config.NUM_INDIVIDUALS}')
    plt.xlabel('generation')
    plt.ylabel('mean fitness')
    fitness_means = []
    x = [i for i in range(len(fitness_data))]
    for i in range(len(fitness_data)):
        fitness_means.append(np.mean(fitness_data[i]))

    plt.plot(x, fitness_means)
    #plt.savefig(f'examples/ca_experiments/fitness_dynamics_CA_g{config.NUM_GENERATIONS}_p{config.NUM_INDIVIDUALS}')

    file_path = 'examples/ca_experiments/last_population.json'
    save_population_to_file(population, file_path)




if __name__ == "__main__":
    main()
