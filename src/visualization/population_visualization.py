from abc import ABC
from typing import List

from nca.core.ecology.population import Population
from visualization.visualization import StatisticsVisualization, Visualization
from visualization.visualization import plt


class PopulationVisualization:

    def __init__(self, population: Population, x_axis, y_axis):
        super().__init__("Population", x_axis=x_axis, y_axis=y_axis)
        self.population = population


class PopulationFitnessVisualization(StatisticsVisualization, PopulationVisualization):

    def prepare(self):
        fitness_values: List[float] = [individual.fitness.value() for individual in self.population]
        self.set_statistics(fitness_values)


class PopulationAgeVisualization(StatisticsVisualization, PopulationVisualization):

    def prepare(self):
        age_values: List[float] = [individual.age for individual in self.population]
        self.set_statistics(age_values)


class NSGA2Visualization(Visualization, PopulationVisualization):

    def prepare(self):
        pass

    def visualize(self):

        number_of_fronts = len(sorted_fronts)
        colors = cm.rainbow(np.linspace(1, 0, number_of_fronts))

        for index, individual in enumerate(self.population.individuals):
            plt.scatter(individual.objectives, s=50, color=colors[index])

        for index, individual in enumerate(self.population.rejected_individuals):
            plt.scatter(individual.objectives, s=50, color=colors[index])

        plt.show()