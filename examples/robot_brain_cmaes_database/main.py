"""Main script for the example."""
import logging

import cma
import config
from base import Base
from evaluator import Evaluator
from experiment import Experiment
from generation import Generation
from genotype import Genotype
from individual import Individual
from population import Population
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group.rng import seed_from_time
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.modular_robot.brains import (
    body_to_actor_and_cpg_network_structure_neighbour,
)
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session


def run_experiment(dbengine: Engine) -> None:
    """
    Run an experiment.

    :param dbengine: An openened database with matching initialize database structure.
    """
    logging.info("----------------")
    logging.info("Start experiment")

    # Create an rng seed.
    rng_seed = seed_from_time() % 2**32  # Cma seed must be smaller than 2**32.

    # Create and save the experiment instance.
    experiment = Experiment(rng_seed=rng_seed)
    logging.info("Saving experiment configuration.")
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # Get the actor and cpg network structure for the body of choice.
    _, cpg_network_structure = body_to_actor_and_cpg_network_structure_neighbour(
        config.BODY
    )

    # Intialize the evaluator that will be used to evaluate robots.
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
    )

    # Initial parameter values for the brain.
    initial_mean = cpg_network_structure.num_connections * [0.5]

    # Initialize the cma optimizer.
    options = cma.CMAOptions()
    options.set("bounds", [-1.0, 1.0])
    options.set("seed", rng_seed)
    opt = cma.CMAEvolutionStrategy(initial_mean, config.INITIAL_STD, options)

    # Run cma for the defined number of generations.
    logging.info("Start optimization process.")
    while opt.countiter < config.NUM_GENERATIONS:
        logging.info(f"Generation {opt.countiter + 1} / {config.NUM_GENERATIONS}.")

        # Get the sampled solutions(parameters) from cma.
        solutions = opt.ask()

        # Evaluate them.
        fitnesses = evaluator.evaluate(solutions)

        # Tell cma the fitnesses.
        # Provide them negated, as cma minimizes but we want to maximize.
        opt.tell(solutions, -fitnesses)

        # From the samples and fitnesses, create a population that we can save.
        population = Population(
            [
                Individual(Genotype(parameters), fitness)
                for parameters, fitness in zip(solutions, fitnesses)
            ]
        )

        # Make it all into a generation and save it to the database.
        generation = Generation(
            experiment=experiment,
            generation_index=opt.countiter,
            population=population,
        )
        logging.info("Saving generation.")
        with Session(dbengine, expire_on_commit=False) as session:
            session.add(generation)
            session.commit()


def main() -> None:
    """Run the program."""
    # Set up standard logging.
    setup_logging(file_name="log.txt")

    # Open the database, only if it does not already exists.
    dbengine = open_database_sqlite(
        config.DATABASE_FILE, open_method=OpenMethod.NOT_EXISTS_AND_CREATE
    )
    # Create the structure of the database.
    Base.metadata.create_all(dbengine)

    # Run the experiment several times.
    for _ in range(config.NUM_REPETITIONS):
        run_experiment(dbengine)


if __name__ == "__main__":
    main()
