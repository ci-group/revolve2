"""
CA representation testing, generates robot with simple survivor selection (top k% of fittest) with
fixed starting gene length (dict of CA rules) and mutation that replaces one random chosen rule with a randomly
generated rule
"""

import logging
from typing import List, Tuple
import numpy as np
import asyncio
import json
import matplotlib.pyplot as plt
from pathlib import Path

from numpy.typing import NDArray

import revolve2.ci_group.rng
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group import terrains, fitness_functions
from revolve2.ci_group.rng import make_rng
from revolve2.ci_group.simulation import create_batch_single_robot_standard
from revolve2.modular_robot import (
    ActiveHinge,
    Body,
    Brick,
    ModularRobot,
    RightAngles,
    ModularRobot,
    get_body_states_single_robot,
)
from revolve2.modular_robot.brains import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot.representations import render_robot
from revolve2.simulators.mujoco import LocalRunner

from revolve2.experimentation.genotypes.tree import (
    TreeGenotype,
    TreeInitParameters,
)


def initialize(num_individuals: int, rng) -> List[TreeGenotype]:
    params = TreeInitParameters(max_depth=5)
    return TreeGenotype.random_individuals(params, num_individuals, rng)


def run_generation(previous_population: List[TreeGenotype], itteration: int, rng):
    """
    Run all runs of an experiment using the provided parameters.

    """
    # Create a list where we will store the success ratio for each repetition.
    generation_fitness = []
    current_population = []

    for itter, individual in enumerate(previous_population):
        g = individual.mutate(rng)
        body = g.develop()

        # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
        brain = BrainCpgNetworkNeighborRandom(rng)
        # Combine the body and brain into a modular robot.
        robot = ModularRobot(body, brain)
        # render_robot(robot, Path() / f"{itteration}_{itter}_robot.png")

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

        current_population.append(g)

    return generation_fitness, current_population


def survivor_selection(generation_fitness, population, percent_survivors):
    # Create a list of (fitness, individual_id) tuples and sort it in descending order
    fitness_with_id = [(fitness, i) for i, fitness in enumerate(generation_fitness)]
    fitness_with_id.sort(reverse=True)

    num_individuals = len(generation_fitness)
    num_survivors = int((percent_survivors / 100) * num_individuals)

    # Select the top N individuals
    top_individuals = [population[i] for _, i in fitness_with_id[:num_survivors]]

    # Determine the number of additional individuals needed to fill the gap
    gap_size = len(population) - len(top_individuals)

    # Insert copies of survived individuals to fill the gap
    for i in range(gap_size):
        top_individuals.append(top_individuals[i % num_survivors])

    return top_individuals


# def save_population_to_file(population: List[TreeGenotype], file_path: Path):
#     population_data = [
#         {str(k): v for k, v in individual._ca_type.rule_set.items()}
#         for individual in population
#     ]
#     with open(file_path, "w") as f:
#         json.dump(population_data, f)


def run_experiment(num_generations: int, num_individuals: int) -> None:
    """Run the simulation."""
    # Set up standard logging.
    # This decides the level of severity of logged messages we want to display.
    # By default this is 'INFO' or more severe, and 'DEBUG' is excluded.
    # Furthermore, a standard message layout is set up.
    # If logging is not set up, important messages can be missed.
    # We also provide a file name, so the log will be written to both the console and that file.
    setup_logging(file_name="log.txt")
    rng = np.random.default_rng()

    fitness_data = []
    population = initialize(num_individuals, rng)

    for i, individual in enumerate(population):
        g = individual.mutate(rng)
        body = g.develop()
        brain = BrainCpgNetworkNeighborRandom(rng)
        robot = ModularRobot(body, brain)
        try:
            render_robot(robot, Path() / f"{i}_initial.png")
        except IndexError:
            pass

    for i in range(num_generations):
        generation_fitness, population_next = run_generation(population, i, rng)
        fitness_data.append([generation_fitness])
        population = survivor_selection(
            generation_fitness.copy(), population_next.copy(), 70
        )

    plt.figure()
    plt.title(f"# generations = {num_generations}, population size = {num_individuals}")
    plt.xlabel("generation")
    plt.ylabel("mean fitness")

    plt.plot(list(range(len(fitness_data))), [np.mean(x) for x in fitness_data])
    plt.savefig(Path() / f"fitness_dynamics_CA_g{num_generations}_p{num_individuals}")

    for i, individual in enumerate(population):
        g = individual.mutate(rng)
        body = g.develop()
        brain = BrainCpgNetworkNeighborRandom(rng)
        robot = ModularRobot(body, brain)

        try:
            render_robot(robot, Path() / f"{i}_final.png")
        except IndexError:
            pass
    # file_path = Path() / "last_population.json"
    # save_population_to_file(population, file_path)
