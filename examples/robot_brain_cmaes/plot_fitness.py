"""Plot the fitness over generations for a previous optimization process."""

import logging

import config
import matplotlib.pyplot as plt
import pandas
from generation import Generation
from parameters import Parameters
from revolve2.core.database import OpenMethod, open_database_sqlite
from sqlalchemy import select


def main() -> None:
    """Run the program."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    db = open_database_sqlite(config.DATABASE_FILE, OpenMethod.OPEN_IF_EXISTS)
    df = pandas.read_sql(
        select(Generation.generation_index, Parameters.fitness).join(Parameters),
        db,
    )
    maxes = df.groupby("generation_index").max().cummax()
    plt.plot(maxes.index, maxes)

    plt.xlabel("Generation Index")
    plt.ylabel("Fitness")
    plt.title("Fitness Over Generations")
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
