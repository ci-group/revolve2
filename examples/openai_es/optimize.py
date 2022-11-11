"""Setup and running of the openai es optimization program."""

import logging
from random import Random

from optimizer import Optimizer
from revolve2.core.database import open_async_database_sqlite
from revolve2.core.optimization import DbId
from revolve2.standard_resources.modular_robots import gecko


async def main() -> None:
    """Run the optimization process."""
    POPULATION_SIZE = 10
    SIGMA = 0.1
    LEARNING_RATE = 0.05
    NUM_GENERATIONS = 3

    SIMULATION_TIME = 10
    SAMPLING_FREQUENCY = 5
    CONTROL_FREQUENCY = 60

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    # random number generator
    rng = Random()
    rng.seed(0)

    # database
    database = open_async_database_sqlite("./database", create=True)

    # unique database identifier for optimizer
    db_id = DbId.root("openaies")

    body = gecko()

    maybe_optimizer = await Optimizer.from_database(
        database=database,
        db_id=db_id,
        rng=rng,
        robot_body=body,
        simulation_time=SIMULATION_TIME,
        sampling_frequency=SAMPLING_FREQUENCY,
        control_frequency=CONTROL_FREQUENCY,
        num_generations=NUM_GENERATIONS,
    )
    if maybe_optimizer is not None:
        logging.info(
            f"Recovered. Last finished generation: {maybe_optimizer.generation_number}."
        )
        optimizer = maybe_optimizer
    else:
        logging.info("No recovery data found. Starting at generation 0.")
        optimizer = await Optimizer.new(
            database=database,
            db_id=db_id,
            rng=rng,
            population_size=POPULATION_SIZE,
            sigma=SIGMA,
            learning_rate=LEARNING_RATE,
            robot_body=body,
            simulation_time=SIMULATION_TIME,
            sampling_frequency=SAMPLING_FREQUENCY,
            control_frequency=CONTROL_FREQUENCY,
            num_generations=NUM_GENERATIONS,
        )

    logging.info("Starting optimization process..")

    await optimizer.run()

    logging.info("Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
