import logging

import config
import matplotlib.pyplot as plt
import pandas
from generation import Generation
from individual import Individual
from population import Population
from revolve2.core.database import OpenCheck, open_database_sqlite
from sqlalchemy import select


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    db = open_database_sqlite(config.DATABASE_FILE, OpenCheck.OPEN_IF_EXISTS)
    df = pandas.read_sql(
        select(Generation.generation_index, Individual.fitness)
        .join(Generation.population)  # type: ignore[misc] # TODO must be possible to type this properly
        .join(Population.individuals),  # type: ignore[misc] # TODO must be possible to type this properly
        db,
    )

    agg = df.groupby("generation_index")["fitness"].agg(["mean", "min", "max"])
    plt.figure()
    plt.plot(agg.index, agg["mean"], label="Average Fitness")
    plt.plot(agg.index, agg["min"], label="Min Fitness")
    plt.plot(agg.index, agg["max"], label="Max Fitness")

    plt.xlabel("Generation Index")
    plt.ylabel("Fitness")
    plt.title("Fitness Over Generations")
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
