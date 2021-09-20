import math

import copy
from typing import List

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.fitness import Fitness
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.evolution.selection.parent_selection import TournamentSelection
from revolve2.nca.core.evolution.selection.selection import SurvivorSelection
from revolve2.nca.core.genome.operators.mutation_operator import ReplaceMutation
from revolve2.nca.core.genome.operators.recombination_operator import OnePointCrossover


def get_circular_fitness(individual: Individual, d3_enabled: bool = False):
    theta = np.random.uniform(0, math.pi / 2)
    omega = np.random.uniform(0, math.pi / 2)
    r = np.random.uniform(0.5, 1)
    fitness = Fitness()
    if d3_enabled:
        fitness.objectives = {"1": r * np.cos(theta) * np.cos(omega), "2": r * np.sin(theta), "3": r * np.cos(theta) * np.sin(omega)}
    else:
        fitness.objectives = {"1": r * np.cos(theta), "2": r * np.sin(theta)}
    return fitness


def get_random_fitness(individual: Individual, d3_enabled: bool = False):
    theta = np.random.normal(-0.5, 0.2)
    omega = np.random.normal(-9, 3)
    fitness = Fitness()
    fitness.objectives = {"omega": omega, "theta": theta}
    return fitness


def get_max_fitness(individual: Individual, d3_enabled: bool = False):
    fitness = Fitness()
    omega = np.random.normal(-9, 3)
    fitness.objectives = {"1": omega, "2": omega}
    return fitness


def get_ones_fitness(individual: Individual):
    fitness = Fitness()
    representation = [copy.deepcopy(element) for element in individual.get_representation()]
    number_of_elements = len(representation)

    difference = np.sum(np.abs(np.ones(number_of_elements) - np.array(representation)))
    fitness.objectives = {"1": -difference, "2": np.random.random()}
    return fitness


def create_individuals(n: int = 10):
    individual_factory = ActorFactory()
    individuals: Actors = individual_factory.create(n)

    return individuals


def evaluate(individuals):
    for individual in individuals:
        individual.value = get_ones_fitness(individual)


class NonDominatedSortingSurvival(SurvivorSelection):

    def __init__(self, minimization: bool = True, debug: bool = False):
        super().__init__()
        self.minimization: bool = minimization
        self.debug: bool = debug

    def order(self, individuals, indexes):
        agents: List[Individual] = []

        for index in indexes:
            agents.append(individuals[index])

        return Actors(agents)

    def algorithm(self, population_individuals: Actors) -> List[Individual]:

        objectives = np.zeros((len(population_individuals), max(1, len(population_individuals[0].get_objectives()))))  # TODO fitnesses is 0

        for index, individual in enumerate(population_individuals):
            if self.minimization:
                objectives[index, :] = individual.get_objectives()
            else:
                objectives[index, :] = [-objective for objective in individual.get_objectives()]

        front_no, max_front = self.nd_sort(objectives, np.inf)
        crowd_dis = self.crowding_distance(objectives, front_no)

        sorted_fronts = self.sort_fronts(objectives, front_no, crowd_dis)

        if self.debug:
            self._visualize(objectives, sorted_fronts, front_no)

        return [population_individuals[sort_index] for sort_index in sorted_fronts]

    def _visualize(self, objectives, sorted_fronts, front_no):
        number_of_fronts = int(max(front_no))
        colors = cm.rainbow(np.linspace(1, 0, number_of_fronts))

        if not self.minimization:   # Correct Maximization trick.
            objectives = - objectives

        ax = None
        if objectives.shape[1] == 3:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        for individual_index in sorted_fronts:
            front_number = int(front_no[individual_index]) - 1
            if objectives.shape[1] == 3:
                ax.scatter(objectives[individual_index, 0], objectives[individual_index, 1], objectives[individual_index, 2], s=10*(number_of_fronts-front_number),
                           color=colors[front_number])
            else:
                plt.scatter(objectives[individual_index, 0], objectives[individual_index, 1], s=5*(number_of_fronts-front_number),
                            color=colors[front_number])

        #for objective in enumerate(objectives[:self.configuration.selection_size]):
        #    plt.scatter(objective[0], objective[1], s=10,
        #                color='white')
        plt.show()

    # adapted from https://www.programmersought.com/article/6084850621/
    # adapted from https://www.programmersought.com/article/6084850621/
    def nd_sort(self, objectives: np.ndarray, max_range: float) -> (List[int], int):
        """
        Non-dominated Sorting algorithm
        :param objectives: objective matrix
        :param max_range:
        :return: (front numbers, biggest front number)
        """
        number_of_individuals, number_of_objectives = objectives.shape[0], objectives.shape[1]
        sorted_matrix = np.lexsort(objectives[:,
                                   ::-1].T)  # loc1 is the position of the new matrix element in the old matrix, sorted from the first column in order
        sorted_objectives = objectives[sorted_matrix]
        inverse_sorted_indexes = sorted_matrix.argsort()  # loc2 is the position of the old matrix element in the new matrix
        frontno = np.ones(number_of_individuals) * np.inf  # Initialize all levels to np.inf
        maxfno = 0  # 0
        while np.sum(frontno < np.inf) < min(max_range,
                                             number_of_individuals):  # The number of individuals assigned to the rank does not exceed the number of individuals to be sorted
            maxfno = maxfno + 1
            for i in range(number_of_individuals):
                if frontno[i] == np.inf:
                    dominated = False
                    for j in range(i):
                        if frontno[j] == maxfno:
                            m = 0
                            flag = 0
                            while m < number_of_objectives and sorted_objectives[i, m] >= sorted_objectives[j, m]:
                                if sorted_objectives[i, m] == sorted_objectives[
                                    j, m]:  # does not constitute a dominant relationship
                                    flag = flag + 1
                                m = m + 1
                            if m >= number_of_objectives and flag < number_of_objectives:
                                dominated = True
                                break
                    if not dominated:
                        frontno[i] = maxfno
        frontno = frontno[inverse_sorted_indexes]
        return frontno, maxfno

    # adapted from https://github.com/ChengHust/NSGA-II/blob/master/crowding_distance.py
    def crowding_distance(self, objectives, front_number):
        """
        The crowding distance of each Pareto front
        :param pop_obj: objective vectors
        :param front_no: front numbers
        :return: crowding distance
        """
        number_of_individuals, number_of_objectives = np.shape(objectives)  # todo x, y?
        crowd_dis = np.zeros(number_of_individuals)

        # Initialize fronts
        front = np.unique(front_number)
        fronts = front[front != np.inf]

        for f in range(len(fronts)):
            front = np.array([k for k in range(len(front_number)) if front_number[k] == fronts[f]])

            # Find bounds
            Fmax = objectives[front, :].max(0)
            Fmin = objectives[front, :].min(0)

            # For each objective sort the front
            for i in range(number_of_objectives):
                rank = np.argsort(objectives[front, i])

                # Initialize Crowding distance
                crowd_dis[front[rank[0]]] = np.inf
                crowd_dis[front[rank[-1]]] = np.inf

                for j in range(1, len(front) - 1):
                    crowd_dis[front[rank[j]]] += (objectives[(front[rank[j + 1]], i)] -
                                                  objectives[(front[rank[j - 1]], i)]) / (Fmax[i] - Fmin[i])

        return crowd_dis

    def sort_fronts(self, objectives, front_no, crowd_dis):
        front_dict = dict() # dictionary indexed by front number inserting objective with crowd distance as tuple

        for objective_index in range(len(objectives)):
            if front_no[objective_index] not in front_dict.keys():
                front_dict[front_no[objective_index]] = [(crowd_dis[objective_index], objective_index)]
            else:
                front_dict[front_no[objective_index]].append((crowd_dis[objective_index], objective_index))

        sorted_fronts: list[int] = []
        sorted_keys = sorted(front_dict.keys())
        for key in sorted_keys:
            front_dict[key].sort(key=lambda x: x[0], reverse=True)
            for element in front_dict[key]:
                sorted_fronts.append(int(element[1]))

        return sorted_fronts


if __name__ == "__main__":

    survival = NonDominatedSortingSurvival(minimization=False, debug=True)
    tournament = TournamentSelection()
    mutation = ReplaceMutation()
    recombination = OnePointCrossover()

    population = Population(create_individuals())

    evaluate(population.individuals)

    for i in range(10):
        new_individuals = [copy.deepcopy(individual) for individual in population.individuals]
        parent_combinations: List[List[Individual]] = tournament.select(new_individuals)

        offspring: Actors = Actors([Individual(mutation(recombination(parents)), parents)
                                    for parents in parent_combinations])

        evaluate(offspring)

        for off, individual in zip(offspring, population.individuals):
            off.genotype = individual.genotype

        new_individuals.extend(offspring)

        accepted_individuals, rejected_individuals = survival.select(new_individuals)

        population.next_generation(accepted_individuals, rejected_individuals)
