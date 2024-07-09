"""Plot fitness over generations for all experiments, averaged."""

import config
import matplotlib.pyplot as plt
import pandas
from database_components import Experiment, Generation, Individual, Population
from sqlalchemy import select

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging


def main() -> None:
    """Run the program."""
    setup_logging()

    dbengine = open_database_sqlite(
        config.DATABASE_FILE, open_method=OpenMethod.OPEN_IF_EXISTS
    )

    df = pandas.read_sql(
        select(
            Experiment.id.label("experiment_id"),
            Generation.generation_index,
            Individual.fitness,
        )
        .join_from(Experiment, Generation, Experiment.id == Generation.experiment_id)
        .join_from(Generation, Population, Generation.population_id == Population.id)
        .join_from(Population, Individual, Population.id == Individual.population_id),
        dbengine,
    )

    agg_per_experiment_per_generation = (
        df.groupby(["experiment_id", "generation_index"])
        .agg({"fitness": ["max", "mean"]})
        .reset_index()
    )
    agg_per_experiment_per_generation.columns = [
        "experiment_id",
        "generation_index",
        "max_fitness",
        "mean_fitness",
    ]

    agg_per_generation = (
        agg_per_experiment_per_generation.groupby("generation_index")
        .agg({"max_fitness": ["mean", "std"], "mean_fitness": ["mean", "std"]})
        .reset_index()
    )
    agg_per_generation.columns = [
        "generation_index",
        "max_fitness_mean",
        "max_fitness_std",
        "mean_fitness_mean",
        "mean_fitness_std",
    ]

    plt.figure()

    # Plot max
    plt.plot(
        agg_per_generation["generation_index"],
        agg_per_generation["max_fitness_mean"],
        label="Max fitness",
        color="b",
    )
    plt.fill_between(
        agg_per_generation["generation_index"],
        agg_per_generation["max_fitness_mean"] - agg_per_generation["max_fitness_std"],
        agg_per_generation["max_fitness_mean"] + agg_per_generation["max_fitness_std"],
        color="b",
        alpha=0.2,
    )

    # Plot mean
    plt.plot(
        agg_per_generation["generation_index"],
        agg_per_generation["mean_fitness_mean"],
        label="Mean fitness",
        color="r",
    )
    plt.fill_between(
        agg_per_generation["generation_index"],
        agg_per_generation["mean_fitness_mean"]
        - agg_per_generation["mean_fitness_std"],
        agg_per_generation["mean_fitness_mean"]
        + agg_per_generation["mean_fitness_std"],
        color="r",
        alpha=0.2,
    )

    plt.xlabel("Generation index")
    plt.ylabel("Fitness")
    plt.title("Mean and max fitness across repetitions with std as shade")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
