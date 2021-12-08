"""
Plot average, min, and max fitness over generations, using the results of the evolutionary optimizer.
Assumes fitness is a float and database is files.
"""

import argparse

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

    best_individuals = [
        max(
            [analyzer.individuals[individual] for individual in generation],
            key=lambda individual: individual.fitness.float,
        )
        for generation in analyzer.generations
    ]

    print([i.fitness.float for i in best_individuals])

    fig, ax = plt.subplots()
    ax.plot(
        [i for i in range(len(best_individuals))],
        [i.fitness.float for i in best_individuals],
    )
    plt.show()


if __name__ == "__main__":
    main()
