"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse

import matplotlib.pyplot as plt
import pandas
from sqlalchemy.future import select

from revolve2.core.database import create_sync_engine_sqlite
from revolve2.core.optimization.ea.evolutionary_optimizer_schema import (
    DbEvolutionaryOptimizer,
    DbEvolutionaryOptimizerGeneration,
    DbEvolutionaryOptimizerIndividual,
)
from revolve2.core.optimization.ea.fitness_float_schema import DbFitnessFloat


def plot(database: str, optimizer_id: int) -> None:
    # open the database
    db = create_sync_engine_sqlite(database)
    # read the optimizer data into a pandas dataframe
    df = pandas.read_sql(
        select(
            DbEvolutionaryOptimizer,
            DbEvolutionaryOptimizerGeneration,
            DbEvolutionaryOptimizerIndividual,
            DbFitnessFloat,
        ).filter(
            (DbEvolutionaryOptimizer.process_id == optimizer_id)
            & (
                DbEvolutionaryOptimizerGeneration.evolutionary_optimizer_id
                == DbEvolutionaryOptimizer.id
            )
            & (
                DbEvolutionaryOptimizerIndividual.evolutionary_optimizer_id
                == DbEvolutionaryOptimizer.id
            )
            & (DbEvolutionaryOptimizerIndividual.fitness_id == DbFitnessFloat.id)
            & (
                DbEvolutionaryOptimizerGeneration.individual_id
                == DbEvolutionaryOptimizerIndividual.individual_id
            )
        ),
        db,
    )
    # calculate max min avg
    describe = (
        df[["generation_index", "fitness"]]
        .groupby(by="generation_index")
        .describe()["fitness"]
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
