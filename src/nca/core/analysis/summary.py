from typing import List

import numpy as np

from nca.core.analysis.statistics import Statistics, fitness_key


class Summary(Statistics):

    def __init__(self):
        super().__init__()

    def analyze(self, statistics_list: List[Statistics]):
        self._find_best_individual(statistics_list)
        self._find_worst_individual(statistics_list)
        self._calculate_mean_fitness(statistics_list)
        self._calculate_median_fitness(statistics_list)
        return self

    def _find_best_individual(self, statistics_list: List[Statistics]):

        number_of_elements = len(statistics_list[0].best_individuals)

        for index in range(1, number_of_elements):

            individuals = []
            for statistic in statistics_list:
                individuals.append(statistic.best_individuals[index])

            self.best_individuals.append(max(individuals, key=fitness_key))

    def _find_worst_individual(self, statistics_list: List[Statistics]):

        number_of_elements = len(statistics_list[0].worst_individuals)

        for index in range(1, number_of_elements):

            individuals = []
            for statistic in statistics_list:
                individuals.append(statistic.worst_individuals[index])

            self.worst_individuals.append(min(individuals, key=fitness_key))

    def _calculate_mean_fitness(self, statistics_list: List[Statistics]):

        number_of_elements = len(statistics_list[0].mean_fitnesses)

        for index in range(1, number_of_elements):

            values = []
            for statistic in statistics_list:
                values.append(statistic.mean_fitnesses[index])

            self.mean_fitnesses.append(np.mean(values))

    def _calculate_median_fitness(self, statistics_list: List[Statistics]):

        number_of_elements = len(statistics_list[0].median_fitnesses)

        for index in range(1, number_of_elements):

            values = []
            for statistic in statistics_list:
                values.append(statistic.median_fitnesses[index])

            self.median_fitnesses.append(np.median(values))