from typing import List

import matplotlib.pyplot as plt

from nca.core.analysis.statistics import Statistics


def plot_statistics(statistics: Statistics):
    plt.plot([individual.fitness for individual in statistics.best_individuals])
    plt.plot([fitness for fitness in statistics.mean_fitnesses])
    plt.plot([fitness for fitness in statistics.median_fitnesses])
    plt.plot([individual.fitness for individual in statistics.worst_individuals])

    plt.title('Statistics')
    plt.ylabel('Fitness')
    plt.legend(["Best", "Mean", "Median", "Worst"])
    plt.show()


def plot_statistics_measures_list(statistics_list: List[Statistics], algorithm_names: List[str]):

    x = range(10)
    y = range(10)

    fig, axes = plt.subplots(nrows=2, ncols=2)



    for statistics in statistics_list:
        axes[0, 0].plot([value for value in statistics.mean_fitnesses])

    axes[0, 0].set(title='Mean')
    axes[0, 0].legend(algorithm_names)

    for statistics in statistics_list:
        axes[0, 1].plot([value for value in statistics.median_fitnesses])

    axes[0, 1].set(title='Median')
    axes[0, 1].legend(algorithm_names)

    for statistics in statistics_list:
        axes[1, 0].plot([individual.fitness for individual in statistics.best_individuals])

    axes[1, 0].set(title='Best Individuals')
    axes[1, 0].legend(algorithm_names)

    for statistics in statistics_list:
        axes[1, 1].plot([individual.fitness for individual in statistics.worst_individuals])

    axes[1, 1].set(title='Worst Individuals')
    axes[1, 1].legend(algorithm_names)

    plt.show()
