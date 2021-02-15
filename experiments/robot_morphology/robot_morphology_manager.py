import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.cluster.vq import whiten

from nca.core.genome.operators.mutation_operator import SwapMutation, InsertMutation
from nca.experiment_manager import ExperimentManager
from revolve.robot.body.robogen.indirect_robogen_initialization import RobogenWordInitialization
from revolve.robot.body.robogen.random_robogen_body import RandomRobogenBodyBuilder

from revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder
from revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype, DirectRobogenGenotype
from revolve.robot.development_request import BodyDevelopmentRequest
from revolve.robot.robot import Robot
from visualization.clustering.binning import MultiDimensionalBinning
from visualization.dimensionality_reduction.pca import PCAReduction

experiment_manager = ExperimentManager()


def robogen_indirect_random():
    features = []
    robots = []
    builder = RobogenBodyBuilder()
    k = 5

    for i in range(1000):
        genotype = IndirectRobogenGenotype()  # RobogenWordInitialization()
        genotype.get_random_representation()

        body_development_request = BodyDevelopmentRequest(0, genotype, None)
        body = builder.create(body_development_request)
        robot: Robot = Robot(genotype, body=body)

        feature_vector = body.morphological_measures.values()
        features.append(feature_vector)
        robots.append(robot)

    # PCAReduction
    #PCAReduction(features)
    MultiDimensionalBinning(features, 10)

def robogen_random():
    features = []
    robots = []
    builder = RobogenBodyBuilder()

    for i in range(10):
        genotype = IndirectRobogenGenotype(RobogenWordInitialization())
        body_development_request = BodyDevelopmentRequest(0, genotype, None)
        body = builder.create(body_development_request)
        robot: Robot = Robot(genotype, body=body)
        body.visualize()
        feature_vector = body.morphological_measures.values()
        features.append(feature_vector)
        robots.append(robot)
        body.visualize()

    # PCAReduction
    PCAReduction(features)


def robogen_direct():
    mutation = InsertMutation()
    features = []
    robots = []
    builder = RobogenBodyBuilder()

    for i in range(1000):
        genotype = DirectRobogenGenotype()
        mutation._mutate(genotype.get_random_representation())
        body_development_request = BodyDevelopmentRequest(0, genotype, None)
        body = builder.create(body_development_request)
        robot: Robot = Robot(genotype, body=body)
        feature_vector = body.morphological_measures.values()
        features.append(feature_vector)
        robots.append(robot)
        body.visualize()

    #PCAReduction(features)
    MultiDimensionalBinning(features, 10)


def robogen_random_body():
    mutation = InsertMutation()
    features = []
    robots = []
    builder = RobogenBodyBuilder()

    for i in range(1000):
        body_builder = RandomRobogenBodyBuilder()
        body = body_builder.develop()
        feature_vector = body.symbolic_measures.values()
        features.append(feature_vector)
    print(features)
    print(np.mean(features, axis=0))

    PCAReduction(features)
    MultiDimensionalBinning(features, 10)


def random():
    features = np.random.normal(0, 1, (10000, 5))

    PCAReduction(features)
    MultiDimensionalBinning(features, 5)


if __name__ == "__main__":
    random()
