import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from nca.core.actor.actors import Actors
from nca.core.actor.fitness import Fitness
from nca.core.actor.individual_factory import ActorFactory
from nca.core.evolution.selection.selection import SurvivorSelection


class NonDominatedSortingSurvival(SurvivorSelection):

    def __init__(self, debug: bool = False):
        super().__init__()
        self.debug: bool = debug

    def __call__(self, population: Actors) -> Actors:

        objectives = np.zeros((len(population), max(1, len(population[0].fitness.keys()))))  # TODO fitnesses is 0

        for index, individual in enumerate(population):
            objectives[index, :] = - individual.get_fitness()

        front_no, max_front = self.nd_sort(objectives, np.inf)
        crowd_dis = self.crowding_distance(objectives, front_no)

        sorted_fronts = self.sort_fronts(objectives, front_no, crowd_dis)

        new_population = population.subset(sorted_fronts[:self.configuration.population_size])
        discarded_population = population.subset(sorted_fronts[self.configuration.population_size:])

        if self.debug:
            self._visualize(objectives, sorted_fronts, new_population, discarded_population)

        return new_population

    def _visualize(self, objectives, sorted_fronts, new_population, discarded_population):
        number_of_fronts = len(sorted_fronts)
        colors = cm.rainbow(np.linspace(1, 0, number_of_fronts))

        ax = None
        if objectives.shape[1] == 3:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        for index, front in enumerate(sorted_fronts):
            if objectives.shape[1] == 3:
                ax.scatter(objectives[front, 0], objectives[front, 1], objectives[front, 2], s=100,
                           color=colors[index])

            else:
                plt.scatter(-objectives[front, 0], -objectives[front, 1], s=50,
                            color=colors[index])

        for individual in new_population:
            plt.scatter(individual.fitness[0], individual.fitness[1], s=5, color='black')

        for individual in discarded_population:
            plt.scatter(individual.fitness[0], individual.fitness[1], s=5, color='white')

    # adapted from https://github.com/ChengHust/NSGA-II/blob/master/nd_sort.py
    def nd_sort(self, objectives, n_sort):
        """
        :rtype:
        :param n_sort:
        :param pop_obj: objective vectors
        :return: [FrontNo, MaxFNo]
        """
        number_of_individuals, number_of_objectives = np.shape(objectives)
        _, inverse_sorted_population = np.unique(objectives[:, 0], return_inverse=True)

        # sorted first objective from high to low
        index = objectives[:, 0].argsort()
        sorted_objectives = objectives[index, :]

        # Prepare inf front for each entry
        front_no = np.inf * np.ones(number_of_individuals, dtype=np.int)
        max_front: int = 0

        # While there are front numbers to assign, continue
        while np.sum(front_no < np.inf) < min(n_sort, len(inverse_sorted_population)):
            max_front += 1

            # for each individual in population
            for current_individual in range(number_of_individuals):
                # Check that its front number is not assigned yet.
                if front_no[current_individual] == np.inf:
                    dominated = False

                    # Count down from the individual index to the last one available.
                    for other_individual in range(current_individual - 1, -1, -1):

                        # compare against others with the same front.
                        if front_no[other_individual] == max_front:
                            # for each objective that is dominating the other candidate
                            m = np.sum(sorted_objectives[current_individual, :] >= sorted_objectives[other_individual, :])
                            dominated = m == number_of_objectives
                            if dominated:
                                break

                    # If it is not dominated, set the current front.
                    if not dominated:
                        front_no[current_individual] = max_front

        return front_no[inverse_sorted_population], max_front

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

        sorted_fronts = []
        sorted_keys = sorted(front_dict.keys())
        for key in sorted_keys:
            front_dict[key].sort(key=lambda x: x[0], reverse=True)
            for element in front_dict[key]:
                sorted_fronts.append(element[1])

        return sorted_fronts


if __name__ == "__main__":
    individual_factory = ActorFactory()
    n = 50
    individuals: Actors = individual_factory.create(n)
    d3_enabled = False
    for individual in individuals:
        theta = np.random.uniform(0, math.pi/2)
        omega = np.random.uniform(0, math.pi/2)
        r = np.random.uniform(0.5, 1)

        if d3_enabled:
            individual.fitness = Fitness([r * np.cos(theta) * np.cos(omega), r * np.sin(theta), r * np.cos(theta) * np.sin(omega)])
        else:
            individual.fitness = Fitness([r * np.cos(theta), r * np.sin(theta)])

    survival = NonDominatedSortingSurvival(debug=True)

    #front_no, max_front, crowd_dis =
    survival(individuals)

    plt.show()

    """
    def pareto_sort(self, population):

        number_of_objectives = population.shape[1]
        population = list(population)

        fronts = []
        front = []

        dominated = False
        for population_index in range(0, len(population)):
            for other_index in range(0, len(population)):

                for front_element in front:
                    m = np.sum(other_index <= front_element)
                    dominated = m == number_of_objectives
                    if dominated:

                        front = np.array(front)


                        sorted_front = front[rank[:, 0]]
                        area = sorted_front[0] - sorted_front[-1]

                        crowding_distance = []
                        for j in range(1, len(front) - 1):
                            crowding_distance.append(np.sum((sorted_front[j-1] - sorted_front[j+1]) / area))

                        print("sort", sorted_front)
                        print("crowding", crowding_distance)

                        fronts.append(front)

                        front = [population_element]

                        break

                if not dominated:
                    front.append(population_element)
                    print("not dominated")
                else:
                    print("dominated")

        if len(front) > 0:
            fronts.append(front)


        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        number_of_fronts = len(fronts)
        colors = cm.rainbow(np.linspace(1, 0, number_of_fronts))
        print(colors)
        index = 0
        for front_index, front in enumerate(fronts):
            front = np.array(front)
            ax.scatter(front[:, 0], front[:, 1], front[:, 2], s=100,
                        color=[colors[front_index] for _ in range(len(front))])
            index += len(front) - 1
        plt.show()
    """