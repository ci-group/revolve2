"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse
from operator import add
from statistics import mean
from typing import List

import matplotlib.pyplot as plt

from revolve2.core.database import dynamic_cast_float
from revolve2.core.database.sqlite import Database
from revolve2.core.optimization.ea import Analyzer as EaAnalyzer


async def plot(databases: List[str]) -> None:
    max_fitness_sum = None
    min_fitness_sum = None
    mean_fitness_sum = None

    for db_file in databases:
        db = await Database.create(db_file)
        with db.begin_transaction() as txn:
            analyzer = EaAnalyzer(txn, db.root)

            max_fitness = [
                max(
                    [
                        dynamic_cast_float(analyzer.individuals[individual].fitness)
                        for individual in generation
                    ],
                )
                for generation in analyzer.generations
            ]

            min_fitness = [
                min(
                    [
                        dynamic_cast_float(analyzer.individuals[individual].fitness)
                        for individual in generation
                    ],
                )
                for generation in analyzer.generations
            ]

            mean_fitness = [
                mean(
                    [
                        dynamic_cast_float(analyzer.individuals[individual].fitness)
                        for individual in generation
                    ],
                )
                for generation in analyzer.generations
            ]
            if max_fitness_sum is None:
                max_fitness_sum = max_fitness
            else:
                assert len(max_fitness_sum) == len(
                    max_fitness
                ), "Not all databases have an equal amount of generations."
                max_fitness_sum = list(map(add, max_fitness_sum, max_fitness))

            if min_fitness_sum is None:
                min_fitness_sum = min_fitness
            else:
                assert len(min_fitness_sum) == len(
                    min_fitness
                ), "Not all databases have an equal amount of generations."
                min_fitness_sum = list(map(add, min_fitness_sum, min_fitness))

            if mean_fitness_sum is None:
                mean_fitness_sum = mean_fitness
            else:
                assert len(mean_fitness_sum) == len(
                    mean_fitness
                ), "Not all databases have an equal amount of generations."
                mean_fitness_sum = list(map(add, mean_fitness_sum, mean_fitness))

    assert max_fitness_sum is not None  # impossible because we always open one database
    assert min_fitness_sum is not None
    assert mean_fitness_sum is not None

    mean_max_fitness = [f / len(databases) for f in max_fitness_sum]
    mean_min_fitness = [f / len(databases) for f in min_fitness_sum]
    mean_avg_fitness = [f / len(databases) for f in mean_fitness_sum]

    x = [i for i in range(len(mean_max_fitness))]

    fig, ax = plt.subplots()
    ax.plot(
        x,
        mean_max_fitness,
    )
    ax.plot(
        x,
        mean_min_fitness,
    )
    ax.plot(
        x,
        mean_avg_fitness,
    )
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.legend(["Max fitness", "Min fitness", "Mean fitess"])
    plt.show()


async def async_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "databases",
        nargs="+",
        help="The databases to make plots for. If more then one database(multiple runs of the same experiment) is provided, their respective plots are averaged into one single plot.",
    )
    args = parser.parse_args()

    await plot(args.databases)


def main() -> None:
    import asyncio

    asyncio.run(async_main())


if __name__ == "__main__":
    main()
