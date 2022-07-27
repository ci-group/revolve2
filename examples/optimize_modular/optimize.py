"""Setup and running of the optimize modular program."""

import logging
from random import Random

import multineat
from genotype import random as random_genotype
from optimizer import Optimizer
from revolve2.core.database import open_async_database_sqlite
from revolve2.core.optimization import ProcessIdGen


async def main() -> None:
    """Run the optimization process."""
    # number of initial mutations for body and brain CPPNWIN networks
    NUM_INITIAL_MUTATIONS = 10

    SIMULATION_TIME = 10
    SAMPLING_FREQUENCY = 5
    CONTROL_FREQUENCY = 60

    POPULATION_SIZE = 10
    OFFSPRING_SIZE = 10
    NUM_GENERATIONS = 3

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    logging.info("Starting optimization")

    # random number generator
    rng = Random()
    rng.seed(6)

    # database
    database = open_async_database_sqlite("./database")

    # process id generator
    process_id_gen = ProcessIdGen()

    # multineat innovation databases
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    initial_population = [
        random_genotype(innov_db_body, innov_db_brain, rng, NUM_INITIAL_MUTATIONS)
        for _ in range(POPULATION_SIZE)
    ]

    process_id = process_id_gen.gen()
    maybe_optimizer = await Optimizer.from_database(
        database=database,
        process_id=process_id,
        innov_db_body=innov_db_body,
        innov_db_brain=innov_db_brain,
        rng=rng,
        process_id_gen=process_id_gen,
    )
    if maybe_optimizer is not None:
        optimizer = maybe_optimizer
    else:
        optimizer = await Optimizer.new(
            database=database,
            process_id=process_id,
            initial_population=initial_population,
            rng=rng,
            process_id_gen=process_id_gen,
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            simulation_time=SIMULATION_TIME,
            sampling_frequency=SAMPLING_FREQUENCY,
            control_frequency=CONTROL_FREQUENCY,
            num_generations=NUM_GENERATIONS,
            offspring_size=OFFSPRING_SIZE,
        )

    logging.info("Starting optimization process..")

    await optimizer.run()

    logging.info("Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
