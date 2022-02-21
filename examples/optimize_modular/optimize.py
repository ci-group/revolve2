import logging
from random import Random

import multineat
from genotype import Genotype
from optimizer import Optimizer

from revolve2.core.database.sqlite import Database as DbSqlite


async def main() -> None:
    # number of initial mutations for body and brain CPPNWIN networks
    NUM_INITIAL_MUTATIONS = 10

    SIMULATION_TIME = 10
    SAMPLING_FREQUENCY = 5
    CONTROL_FREQUENCY = 5

    POPULATION_SIZE = 10
    OFFSPRING_SIZE = 10
    NUM_GENERATIONS = 3

    logging.basicConfig(
        level=logging.DEBUG,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    logging.info(f"Starting optimization")

    # random number generator
    rng = Random()
    rng.seed(5)

    # database
    database = await DbSqlite.create(f"database")

    # multineat innovation databases
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    initial_population = [
        Genotype.random(innov_db_body, innov_db_brain, rng, NUM_INITIAL_MUTATIONS)
        for _ in range(POPULATION_SIZE)
    ]

    ep = await Optimizer.create(
        database,
        initial_population=initial_population,
        initial_fitness=None,
        rng=rng,
        innov_db_body=innov_db_body,
        innov_db_brain=innov_db_brain,
        simulation_time=SIMULATION_TIME,
        sampling_frequency=SAMPLING_FREQUENCY,
        control_frequency=CONTROL_FREQUENCY,
        num_generations=NUM_GENERATIONS,
        population_size=POPULATION_SIZE,
        offspring_size=OFFSPRING_SIZE,
    )

    logging.info("Starting optimization process..")

    await ep.run()

    logging.info(f"Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
