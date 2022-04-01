"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse

import matplotlib.pyplot as plt
import pandas
from sqlalchemy.future import select

from revolve2.core.database import open_database_sqlite
from revolve2.core.database.serializers import DbFloat
from revolve2.core.optimization.ec.ea import (
    DbEAOptimizer,
    DbEAOptimizerGeneration,
    DbEAOptimizerIndividual,
)


def plot(database: str, optimizer_id: int) -> None:
    # open the database
    db = open_database_sqlite(database)
    # read the optimizer data into a pandas dataframe
    df = pandas.read_sql(
        select(
            DbEAOptimizer,
            DbEAOptimizerGeneration,
            DbEAOptimizerIndividual,
            DbFloat,
        ).filter(
            (DbEAOptimizer.process_id == optimizer_id)
            & (DbEAOptimizerGeneration.ea_optimizer_id == DbEAOptimizer.id)
            & (DbEAOptimizerIndividual.ea_optimizer_id == DbEAOptimizer.id)
            & (DbEAOptimizerIndividual.fitness_id == DbFloat.id)
            & (
                DbEAOptimizerGeneration.individual_id
                == DbEAOptimizerIndividual.individual_id
            )
        ),
        db,
    )
    # calculate max min avg
    describe = (
        df[["generation_index", "value"]]
        .groupby(by="generation_index")
        .describe()["value"]
    )
    mean = describe[["mean"]].values.squeeze()
    std = describe[["std"]].values.squeeze()

    # plot max min mean, std
    describe[["max", "mean", "min"]].plot()
    plt.fill_between(range(len(mean)), mean - std, mean + std)
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "database",
        type=str,
        help="The database to plot.",
    )
    parser.add_argument(
        "optimizer_id", type=int, help="The id of the ea optimizer to plot."
    )
    args = parser.parse_args()

    plot(args.database, args.optimizer_id)


if __name__ == "__main__":
    main()
