"""
Plot average, min, and max fitness over generations using the results of the evolutionary optimizer.

Assumes fitnesses is a floats.
Installed as ``revolve2_plot_ea_fitness_float``.
See ``revolve2_plot_ea_fitness_float --help`` for usage.
"""

import argparse

import matplotlib.pyplot as plt
import pandas
from revolve2.core.database import open_database_sqlite
from revolve2.core.database.serializers import DbFloat
from revolve2.core.optimization import DbId
from revolve2.core.optimization.ea.generic_ea import (
    DbEAOptimizer,
    DbEAOptimizerGeneration,
    DbEAOptimizerIndividual,
)
from sqlalchemy.future import select


def plot(database: str, db_id: DbId) -> None:
    """
    Plot fitness as described at the top of this file.

    :param database: Database where the data is stored.
    :param db_id: Id of the evolutionary process to plot.
    """
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
            (DbEAOptimizer.db_id == db_id.fullname)
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
    print(df)
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
    """Run this file as a command line tool."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "database",
        type=str,
        help="The database to plot.",
    )
    parser.add_argument("db_id", type=str, help="The id of the ea optimizer to plot.")
    args = parser.parse_args()

    plot(args.database, DbId(args.db_id))


if __name__ == "__main__":
    main()
