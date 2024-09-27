"""Main script for the example."""

import logging

from pyrr import Vector3

from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulation.scene import Pose
from revolve2.simulators.mujoco_simulator import LocalSimulator
from revolve2.standards import terrains
from revolve2.standards.simulation_parameters import make_standard_batch_parameters
from genotype import Genotype  
from individual import Individual 
import multineat
import config
from evaluator import Evaluator
import random
import numpy as np

from revolve2.standards.morphological_novelty_metric import get_novelty_from_population
from revolve2.standards.morphological_measures import MorphologicalMeasures
from sklearn.neighbors import KDTree
import numpy as np
from scipy.spatial import distance

def calculate_morphological_features(robot: ModularRobot) -> dict:
    """
    Calculate the morphological features for a given robot using the MorphologicalMeasures class.

    :param robot: A ModularRobot object.
    :returns: A dictionary with the calculated morphological features.
    """
    measures = MorphologicalMeasures(robot.body) # Explore this class and the referenced paper to learn more about the traits
    features = {
        'symmetry': measures.symmetry,
        'proportion': measures.proportion_2d if measures.is_2d else 0,  # Use 0 for 3D robots
        'coverage': measures.coverage,
        'extremities_prop': measures.limbs,  
        'extensiveness_prop': measures.length_of_limbs,  
        'hinge_prop': measures.num_active_hinges / measures.num_modules if measures.num_modules > 0 else 0,
        'hinge_ratio': measures.num_active_hinges / measures.num_bricks if measures.num_bricks > 0 else 0,
        'branching_prop': measures.branching,  
    }
    return features

def calculate_euclidean_diversity(morphological_features: list[dict]) -> float:
    """
    Calculate the Euclidean diversity of the population using morphological features.

    :param morphological_features: List of morphological features for each robot in the population.
    :returns: The average Euclidean diversity of the population.
    """
    n = len(morphological_features)
    total_distance = 0
    count = 0

    # Convert feature dictionaries to feature vectors
    feature_vectors = [list(features.values()) for features in morphological_features]

    # Calculate pairwise Euclidean distances
    for i in range(n):
        for j in range(i + 1, n):
            dist = distance.euclidean(feature_vectors[i], feature_vectors[j])
            total_distance += dist
            count += 1

    # Average Euclidean distance across all pairs
    avg_euclidean_diversity = total_distance / count if count > 0 else 0
    return avg_euclidean_diversity

def calculate_kdtree_diversity(morphological_features: list[dict]) -> float:
    """
    Calculate the diversity of the population using KDTree for nearest neighbors.

    :param morphological_features: List of morphological features for each robot in the population.
    :returns: The average KDTree-based diversity of the population.
    """
    # Convert feature dictionaries to feature vectors and buld a Kdtree
    feature_vectors = [list(features.values()) for features in morphological_features]
    kdtree = KDTree(feature_vectors, leaf_size=30, metric='euclidean')

    # Compute the distances of each robot to its k nearest neighbors
    k = len(morphological_features) - 1  # Using k-1 because the nearest neighbor is the point itself
    distances, _ = kdtree.query(feature_vectors, k=k)

    # Calculate the average diversity as the mean distance to nearest neighbors
    avg_kdtree_diversity = np.mean([np.mean(dist) for dist in distances])

    return avg_kdtree_diversity

def main() -> None:
    """Run the simulation."""

    # Set up logging.
    setup_logging()

    # Set up the random number generator and databases.
    rng = make_rng_time_seed()
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    # Create an initial population.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")

    # You can choose to not evaluate the robots if all you want is to visualize the morphologies or compute diversity to save time
    if config.EVALUATE:
        initial_fitnesses = Evaluator(headless=True, num_simulators=config.NUM_SIMULATORS).evaluate(initial_genotypes)
    else:
        initial_fitnesses = [random.uniform(0.0, 1.0) for _ in range(len(initial_genotypes))]

    # Create a population of individuals, combining genotype with fitness.
    population = [
        Individual(genotype, fitness)
        for genotype, fitness in zip(initial_genotypes, initial_fitnesses, strict=True)
    ]

    # Create the robot bodies from the genotypes of the population
    robots = [individual.genotype.develop(config.VISUALIZE_MAP) for individual in population]

    # Calculate the morphological features for each robot in the population
    morphological_features = []
    for robot in robots:
        features = calculate_morphological_features(robot)
        morphological_features.append(features)

    # Calculate the Euclidean-based morphological diversity of the population
    euclidean_diversity = calculate_euclidean_diversity(morphological_features)
    print(f"Euclidean-based morphological diversity of the population: {euclidean_diversity}")

    # Calculate the KDTree-based morphological diversity of the population
    kdtree_diversity = calculate_kdtree_diversity(morphological_features)
    print(f"KDTree-based morphological diversity of the population: {kdtree_diversity}")

    # Now we can create a scene and add the robots by mapping the genotypes to phenotypes
    scene = ModularRobotScene(terrain=terrains.flat())
    i = 0
    for individual in robots:
        # By changing the value of "VISUALIZE_MAP" to true you can plot the body creation process
        scene.add_robot(individual, pose=Pose(Vector3([i, 0.0, 0.0])))
        i += 1

    # Declare the simulators and simulation parameters. 
    simulator = LocalSimulator(viewer_type="custom", headless=False)
    batch_parameters = make_standard_batch_parameters()
    batch_parameters.simulation_time = 1000
    batch_parameters.simulation_timestep = 0.01
    batch_parameters.sampling_frequency = 10

    # Finally simulate the scenes to inspect the resulting morphologies
    simulate_scenes(
        simulator=simulator,
        batch_parameters=batch_parameters,
        scenes=scene,
    )


if __name__ == "__main__":
    main()



