"""Main script for the example."""

import logging

import cma
import config
from database_components import (
    Base,
    Experiment,
    Generation,
    Genotype,
    Individual,
    Population,
)
from evaluator import Evaluator
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import seed_from_time
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain.cpg import (
    active_hinges_to_cpg_network_structure_neighbor,
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

    # Find all active hinges in the body
    active_hinges = config.BODY.find_modules_of_type(ActiveHinge)

    # Create a structure for the CPG network from these hinges.
    # This also returns a mapping between active hinges and the index of there corresponding cpg in the network.
    (
        cpg_network_structure,
        output_mapping,
    ) = active_hinges_to_cpg_network_structure_neighbor(active_hinges)

    # Intialize the evaluator that will be used to evaluate robots.
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
        output_mapping=output_mapping,
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
            individuals=[
                Individual(genotype=Genotype(parameters), fitness=fitness)
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
    # Set up logging.
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
