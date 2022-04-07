"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse

import matplotlib.pyplot as plt
import pandas
from sqlalchemy.future import select

from revolve2.core.database import open_database_sqlite
from revolve2.core.optimization.ec.openai_es import DbOpenaiESOptimizerIndividual


def plot(database: str, process_id: int) -> None:
    # open the database
    db = open_database_sqlite(database)
    # read the optimizer data into a pandas dataframe
    df = pandas.read_sql(
        select(DbOpenaiESOptimizerIndividual).filter(
            DbOpenaiESOptimizerIndividual.process_id == process_id
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
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "database",
        type=str,
        help="The database to plot.",
    )
    parser.add_argument(
        "process_id", type=int, help="The id of the ea optimizer to plot."
    )
    args = parser.parse_args()

    plot(args.database, args.process_id)


if __name__ == "__main__":
    main()
