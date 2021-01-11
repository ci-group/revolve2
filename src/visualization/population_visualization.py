
from nca.core.ecology.population import Population
from visualization.visualization import StatisticsVisualization, Visualization
from visualization.visualization import plt


class PopulationVisualization:

    def __init__(self, population: Population, x_axis="x", y_axis="y"):
        super().__init__("Population", x_axis=x_axis, y_axis=y_axis)
        self.population: Population = population


class PopulationFitnessVisualization(PopulationVisualization, StatisticsVisualization):

    def prepare(self):
        self.x_axis = "generation"
        self.y_axis = "fitness"
        self.statistics = self.population.generational_statistics['fitness']


class PopulationAgeVisualization(StatisticsVisualization, PopulationVisualization):

    def prepare(self):
        self.statistics = self.population.generational_statistics['age']


class SpeciationVisualization(StatisticsVisualization, PopulationVisualization):
    pass


class NSGA2Visualization(Visualization, PopulationVisualization):

    def prepare(self):
        pass

    def visualize(self):

        for index, individual in enumerate(self.population.individuals):
            plt.scatter(individual.objectives, s=50, color="green")

        for index, individual in enumerate(self.population.rejected_individuals):
            plt.scatter(individual.objectives, s=50, color="red")
