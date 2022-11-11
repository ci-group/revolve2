"""Setup and running of the knapsack optimization program."""

from __future__ import annotations

import logging
from random import Random

from genotype import random as random_genotype
from item import Item
from optimizer import Optimizer
from revolve2.core.database import open_async_database_sqlite
from revolve2.core.optimization import DbId


async def main() -> None:
    """Run the optimization process."""
    POPULATION_SIZE = 100
    OFFSPRING_SIZE = 100
    NUM_GENERATIONS = 25

    INITIAL_HAS_ITEM_PROB = 0.5
    MAX_WEIGHT = 300

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    logging.info("Starting optimization")

    # random number generator
    rng = Random()
    rng.seed(100)

    # create 100 random items
    items = [Item(rng.randrange(0, 100), rng.randrange(0, 100)) for _ in range(100)]

    # database
    database = open_async_database_sqlite("./database", create=True)

    # unique database identifier for optimizer
    db_id = DbId.root("simpleopt")

    initial_population = [
        random_genotype(rng, INITIAL_HAS_ITEM_PROB, len(items))
        for _ in range(POPULATION_SIZE)
    ]

    maybe_optimizer = await Optimizer.from_database(
        database=database,
        db_id=db_id,
        rng=rng,
        items=items,
        max_weight=MAX_WEIGHT,
        num_generations=NUM_GENERATIONS,
    )
    if maybe_optimizer is not None:
        optimizer = maybe_optimizer
    else:
        optimizer = await Optimizer.new(
            database=database,
            db_id=db_id,
            offspring_size=OFFSPRING_SIZE,
            initial_population=initial_population,
            rng=rng,
            items=items,
            max_weight=MAX_WEIGHT,
            num_generations=NUM_GENERATIONS,
        )

    logging.info("Starting optimization process..")

    await optimizer.run()

    logging.info("Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
