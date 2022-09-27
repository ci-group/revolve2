"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.

Assumes fitness is a float and database is files.
See program help for what inputs to provide.
"""

import argparse

import matplotlib.pyplot as plt
import pandas
from revolve2.core.database import open_database_sqlite
from revolve2.core.optimization import DbId
from revolve2.core.optimization.ea.openai_es import DbOpenaiESOptimizerIndividual
from sqlalchemy.future import select


def plot(database: str, db_id: DbId) -> None:
    """
    Do the actual plotting.

    :param database: The database with the results.
    :param db_id: The id of the ea optimizer to plot.
    """
    # open the database
    db = open_database_sqlite(database)
    # read the optimizer data into a pandas dataframe
    df = pandas.read_sql(
        select(DbOpenaiESOptimizerIndividual).filter(
            DbOpenaiESOptimizerIndividual.db_id == db_id
        ),
        db,
    )
    # calculate max min avg
    describe = df[["gen_num", "fitness"]].groupby(by="gen_num").describe()["fitness"]
    mean = describe[["mean"]].values.squeeze()
    std = describe[["std"]].values.squeeze()

    # plot max min mean, std
    describe[["max", "mean", "min"]].plot()
    plt.fill_between(range(1, len(mean) + 1), mean - std, mean + std)
    plt.show()


def main() -> None:
    """Run the program."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "database",
        type=str,
        help="The database to plot.",
    )
    parser.add_argument(
        "db_id",
        type=int,
        help="The id of the ea optimizer to plot.",
    )
    args = parser.parse_args()

    plot(args.database, DbId(args.db_id))


if __name__ == "__main__":
    main()
