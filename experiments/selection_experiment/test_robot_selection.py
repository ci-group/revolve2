import unittest

import numpy as np

from revolve2.abstract.configurations import InitializationConfiguration, RepresentationConfiguration
from experiments.selection_experiment.measures_read import get_robot_measures
from experiments.selection_experiment.plot_measures import plot_measures
from revolve2.nca.core.actor.fitness_evaluation import FitnessEvaluation
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration
from revolve2.nca.core.genome.operators.initialization import UniqueIntegerInitialization
from revolve2.nca.core.genome.operators.mutation_operator import UniqueReplaceMutation
from revolve2.nca.core.genome.operators.recombination_operator import UniqueElementCrossover
from revolve2.nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve2.nca.evolution import Evolution
from visualization.population_visualization import PopulationFitnessVisualization
from visualization.visualization import time_series_visualization

number_of_robots = 20

measures = ['joints', 'proportion', 'hinge_count', 'size', 'branching', 'limbs', 'coverage', 'symmetry']
number_of_objectives = len(measures)
number_of_selected_robots = 9


class RobotSelectionFitness(FitnessEvaluation):

    def __init__(self, robot_samples=True):
        if robot_samples:
            self.measure_values, _ = get_robot_measures(measures)
        else:
            self.measure_values = np.random.random((number_of_robots, number_of_objectives))

    def __call__(self, individual: Individual):
        robot_indexes = individual.get_representation()
        return sum(np.std(self.measure_values[robot_indexes], axis=0))


class TestRobotSelection(unittest.TestCase):

    def test_create(self):
        # Initialize the representation between 0 and 20 robots
        initialization_configuration = InitializationConfiguration(0, number_of_robots)
        representation_initialization = UniqueIntegerInitialization(initialization_configuration)
        # Create the Genetic Algorithm configuration for doing evolution
        algorithm_configuration = GeneticAlgorithmConfiguration(mutation=UniqueReplaceMutation(),
                                                                recombination=UniqueElementCrossover(),
                                                                initialization_type=representation_initialization)

        # Create the evolutionary framework based on the EA configuration and representation.
        representation = ValuedRepresentation(representation_initialization, configuration=RepresentationConfiguration(size=number_of_selected_robots))
        individual_factory = ActorFactory(representation)
        evolution = Evolution(evolutionary_configuration=algorithm_configuration,
                              fitness_evaluation=RobotSelectionFitness(robot_samples=True),
                              individual_factory=individual_factory)

        population_ecology = evolution.evolve()

        time_series_visualization(population_ecology.management.population.population_metrics)
        viz = PopulationFitnessVisualization(population_ecology.management.population)
        viz.prepare()
        viz.visualize()

        best_individual_index = population_ecology.management.population.individuals.find_typical_individuals()['max']
        best_selection = population_ecology.management.population.individuals[best_individual_index].get_representation()
        print("Best Individual", best_selection)

        robot_names = ['bulkyA', 'zappa', 'park', 'babyB', 'penguin', 'snake', 'garrix', 'babyA', 'bohmer', 'salamander', 'spider', 'blokky', 'queen', 'frank', 'turtle', 'gecko', 'linkin', 'pentapede', 'insect', 'martin']
        selected_robots = np.array(robot_names)[best_selection]
        print(np.array(robot_names)[best_selection])

        plot_measures(measures, selected_robots)
        self.assertTrue(True)
