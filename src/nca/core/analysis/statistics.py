import numpy as np

from nca.core.ecology.population import Population


def fitness_key(individual):
    return individual.get_fitness()


class Statistics:

    def __init__(self):
        self.best_individuals = []
        self.mean_fitnesses = []
        self.median_fitnesses = []
        self.worst_individuals = []

    def log(self, population: Population):
        self._find_best_individual(population)
        self._calculate_mean_fitness(population)
        self._calculate_median_fitness(population)
        self._find_worst_individual(population)

    def _find_best_individual(self, population: Population):
        best_individual = max(population.individuals, key=fitness_key)
        self.best_individuals.append(best_individual)

    def _calculate_mean_fitness(self, population: Population):
        self.mean_fitnesses.append(np.mean([individual.get_fitness() for individual in population.individuals]))

    def _calculate_median_fitness(self, population: Population):
        self.median_fitnesses.append(np.median([individual.get_fitness() for individual in population.individuals]))

    def _find_worst_individual(self, population: Population):
        worst_individual = min(population.individuals, key=fitness_key)
        self.worst_individuals.append(worst_individual)


