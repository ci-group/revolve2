"""Plot fitness over generations for all experiments, averaged."""

import config
import matplotlib.pyplot as plt
import pandas
from experiment import Experiment
from generation import Generation
from individual import Individual
from population import Population
from sqlalchemy import select

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging


def main() -> None:
    """Run the program."""
    setup_logging()

    # The the database.
    dbengine = open_database_sqlite(
        config.DATABASE_FILE, open_method=OpenMethod.OPEN_IF_EXISTS
    )

    # Read data from the database into a pandas dataframe.
    # The loaded dataframe should have the columns `experiment_id`, `generation_index`, and `fitness`.
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

    # Calculate the max and mean fitness within in generation, for each experiment seperately.
    agg_per_experiment_per_generation = (
        df.groupby(["experiment_id", "generation_index"])
        .agg({"fitness": ["max", "mean"]})
        .reset_index()
    )

    # Give specific names to the new max and mean columns
    agg_per_experiment_per_generation.columns = [
        "experiment_id",
        "generation_index",
        "max_fitness",
        "mean_fitness",
    ]

    # For the mean and max fitnesses, calculate the mean and standard deviation with respect to the seperate experiments.
    agg_per_generation = (
        agg_per_experiment_per_generation.groupby("generation_index")
        .agg({"max_fitness": ["mean", "std"], "mean_fitness": ["mean", "std"]})
        .reset_index()
    )

    # Give these proper names as well.
    agg_per_generation.columns = [
        "generation_index",
        "max_fitness_mean",
        "max_fitness_std",
        "mean_fitness_mean",
        "mean_fitness_std",
    ]

    # Next, create a plot.
    plt.figure()

    # Plot mean and std of the max fitness
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

    # Plot mean and std of the mean fitness
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
