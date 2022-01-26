import logging
from random import Random
from revolve2.core.database.sqlite import Database as DbSqlite


async def main():
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
    rng.seed(100)

    # database
    database = await DbSqlite.create(f"database")

    # here you will add your initial population and optimizer later
    # ...

    logging.info("Starting optimization process..")

    # await ep.run()

    logging.info(f"Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
