"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse
from statistics import mean

import matplotlib.pyplot as plt
from revolve2.core.database.files import Database
from revolve2.core.database.view import AnyView
from revolve2.core.optimization.ea import Analyzer as EaAnalyzer


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    args = parser.parse_args()

    database = Database(args.database)
    analyzer = EaAnalyzer(AnyView(database, database.root()))

    max_fitness = [
        max(
            [
                analyzer.individuals[individual].fitness.float
                for individual in generation
            ],
        )
        for generation in analyzer.generations
    ]

    min_fitness = [
        min(
            [
                analyzer.individuals[individual].fitness.float
                for individual in generation
            ],
        )
        for generation in analyzer.generations
    ]

    mean_fitness = [
        mean(
            [
                analyzer.individuals[individual].fitness.float
                for individual in generation
            ],
        )
        for generation in analyzer.generations
    ]

    x = [i for i in range(len(analyzer.generations))]

    fig, ax = plt.subplots()
    ax.plot(
        x,
        max_fitness,
    )
    ax.plot(
        x,
        min_fitness,
    )
    ax.plot(
        x,
        mean_fitness,
    )
    plt.show()


if __name__ == "__main__":
    main()
