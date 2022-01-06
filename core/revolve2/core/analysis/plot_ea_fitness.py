"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse
from statistics import mean

import matplotlib.pyplot as plt
from revolve2.core.database.sqlite import Database
from revolve2.core.optimization.ea import Analyzer as EaAnalyzer
from revolve2.core.optimization.ea.analyzer import Generation


async def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "databases",
        nargs="+",
        help="The databases to make plots for. If more then one database(multiple runs of the same experiment) is provided, their respective plots are averaged into one single plot.",
    )
    args = parser.parse_args()

    max_fitnesses = []
    min_fitnesses = []
    mean_fitnesses = []

    for db_file in args.databases:
        db = await Database.create(db_file)
        with db.begin_transaction() as txn:
            analyzer = EaAnalyzer(txn, db.root)

            max_fitness = [
                max(
                    [
                        analyzer.individuals[individual].fitness
                        for individual in generation
                    ],
                )
                for generation in analyzer.generations
            ]

            min_fitness = [
                min(
                    [
                        analyzer.individuals[individual].fitness
                        for individual in generation
                    ],
                )
                for generation in analyzer.generations
            ]

            mean_fitness = [
                mean(
                    [
                        analyzer.individuals[individual].fitness
                        for individual in generation
                    ],
                )
                for generation in analyzer.generations
            ]
            max_fitnesses.append(max_fitness)
            min_fitnesses.append(min_fitness)
            mean_fitnesses.append(mean_fitness)

    assert all(
        len(max_fitnesses[0]) == len(x) for x in max_fitnesses
    ), "Not all databases have an equal amount of generations."

    mean_max_fitness = [mean(x) for x in list(map(list, zip(*max_fitnesses)))]
    mean_min_fitness = [mean(x) for x in list(map(list, zip(*min_fitnesses)))]
    mean_avg_fitness = [mean(x) for x in list(map(list, zip(*mean_fitnesses)))]

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
    plt.show()


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
