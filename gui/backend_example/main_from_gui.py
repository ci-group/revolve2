"""Main script for the example."""

import logging
from datetime import datetime

import config
import multineat
from database_components import (
    Base,
    Experiment,
    Generation,
    Genotype,
    Individual,
    Population,
)
from evaluator import Evaluator
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session

from reproducer_methods import CrossoverReproducer
from selector_methods import ParentSelector, SurvivorSelector

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.evolution import ModularRobotEvolution
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng, seed_from_time

import sys

def run_experiment(dbengine: Engine) -> None:
    """
    Run an experiment.

    :param dbengine: An openened database with matching initialize database structure.
    """
    logging.info("----------------")
    logging.info("Start experiment")

    # Set up the random number generator.
    rng_seed = seed_from_time()
    rng = make_rng(rng_seed)


    terrain = sys.argv[1]
    fitness_function = sys.argv[2]
    parent_selection_function = sys.argv[3]
    parent_selection_function_params = sys.argv[4]
    survivor_selection_function = sys.argv[5]
    survivor_selection_function_params = sys.argv[6]

    # create dictionaries from selection params
    parent_selection_function_params = eval(parent_selection_function_params)
    survivor_selection_function_params = eval(survivor_selection_function_params)


    # Create and save the experiment instance.
    experiment = Experiment(rng_seed=rng_seed, terrain=terrain)
    logging.info("Saving experiment configuration.")
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # CPPN innovation databases.
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    """
    Here we initialize the components used for the evolutionary process.
    
    - evaluator: Allows us to evaluate a population of modular robots.
    - parent_selector: Allows us to select parents from a population of modular robots.
    - survivor_selector: Allows us to select survivors from a population.
    - crossover_reproducer: Allows us to generate offspring from parents.
    - modular_robot_evolution: The evolutionary process as a object that can be iterated.
    """
    evaluator = Evaluator(headless=True, num_simulators=config.NUM_SIMULATORS, terrain=terrain, fitness_function=fitness_function)
    parent_selector = ParentSelector(offspring_size=config.OFFSPRING_SIZE, rng=rng, selection_func=parent_selection_function, selection_params=parent_selection_function_params)
    survivor_selector = SurvivorSelector(rng=rng, selection_func=survivor_selection_function, selection_params=survivor_selection_function_params)
    crossover_reproducer = CrossoverReproducer(rng=rng, innov_db_body=innov_db_body, innov_db_brain=innov_db_brain)

    modular_robot_evolution = ModularRobotEvolution(
                                parent_selection=parent_selector,
                                survivor_selection=survivor_selector,
                                evaluator=evaluator,
                                reproducer=crossover_reproducer,
                            )

    # Create an initial population, as we cant start from nothing.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")
    initial_fitnesses = evaluator.evaluate(initial_genotypes)

    # Create a population of individuals, combining genotype with fitness.
    population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness)
            for genotype, fitness in zip(
                initial_genotypes, initial_fitnesses, strict=True
            )
        ]
    )

    # Finish the zeroth generation and save it to the database.
    generation = Generation(
        experiment=experiment, generation_index=0, population=population
    )
    save_to_db(dbengine, generation)

    # Start the actual optimization process.
    logging.info("Start optimization process.")
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )

        # Here we iterate the evolutionary process using the step.
        population = modular_robot_evolution.step(population)

        # Make it all into a generation and save it to the database.
        generation = Generation(
            experiment=experiment,
            generation_index=generation.generation_index + 1,
            population=population,
        )
        save_to_db(dbengine, generation)


def main() -> None:
    """Run the program."""
    # Set up logging.
    setup_logging(file_name="log.txt")

    # Open the database, only if it does not already exists.
    dbengine = open_database_sqlite(
        "gui/resources/databases/"+f"{datetime.now()}"+config.DATABASE_FILE, open_method=OpenMethod.NOT_EXISTS_AND_CREATE
    )
    # Create the structure of the database.
    Base.metadata.create_all(dbengine)

    # Run the experiment several times.
    for _ in range(config.NUM_REPETITIONS):
        run_experiment(dbengine)


def save_to_db(dbengine: Engine, generation: Generation) -> None:
    """
    Save the current generation to the database.

    :param dbengine: The database engine.
    :param generation: The current generation.
    """
    logging.info("Saving generation.")
    with Session(dbengine, expire_on_commit=False) as session:
        session.add(generation)
        session.commit()


if __name__ == "__main__":
    main()
