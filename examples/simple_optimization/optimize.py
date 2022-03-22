from __future__ import annotations

import logging
from random import Random

from genotype import Genotype
from item import Item
from optimizer import Optimizer

from revolve2.core.database import open_database_sqlite
from revolve2.core.optimization import ProcessIdGen


async def main() -> None:
    POPULATION_SIZE = 100
    OFFSPRING_SIZE = 100
    NUM_GENERATIONS = 25

    INITIAL_HAS_ITEM_PROB = 0.5

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    logging.info(f"Starting optimization")

    # random number generator
    rng = Random()
    rng.seed(100)

    # create 100 random items
    items = [Item(rng.randrange(0, 100), rng.randrange(0, 100)) for _ in range(100)]

    # database
    database = open_database_sqlite("./database")

    # process id generator
    process_id_gen = ProcessIdGen()

    initial_population = [
        Genotype.random(rng, INITIAL_HAS_ITEM_PROB, len(items))
        for _ in range(POPULATION_SIZE)
    ]

    maybe_optimizer = await Optimizer.from_database(
        database=database,
        process_id=0,
        process_id_gen=process_id_gen,
        rng=rng,
        items=items,
        num_generations=NUM_GENERATIONS,
    )
    if maybe_optimizer is not None:
        optimizer = maybe_optimizer
    else:
        optimizer = await Optimizer.new(
            database=database,
            process_id=0,
            process_id_gen=process_id_gen,
            offspring_size=OFFSPRING_SIZE,
            initial_population=initial_population,
            rng=rng,
            items=items,
            num_generations=NUM_GENERATIONS,
        )

    logging.info("Starting optimization process..")

    await optimizer.run()

    logging.info(f"Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
