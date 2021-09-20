import numpy as np

from revolve2.nca.core.genome.operators.mutation_operator import SwapMutation, InsertMutation
from revolve2.nca.experiment_manager import ExperimentManager
from revolve2.revolve.robot.body.robogen.indirect_robogen_initialization import RobogenWordInitialization
from revolve2.revolve.robot.body.robogen.random_robogen_body import RandomRobogenBodyBuilder

from revolve2.revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder
from revolve2.revolve.robot.body.robogen.robogen_genotype import IndirectRobogenGenotype, DirectRobogenGenotype
from revolve2.revolve.robot.body.robogen.symbolic_measures import MorphologicalMeasures
from revolve2.revolve.robot.development_request import BodyDevelopmentRequest
from revolve2.revolve.robot.robot import Robot
from analysis.clustering.binning import MultiDimensionalBinning
from analysis.dimensionality_reduction.pca import PCAReduction

from src.correlation.correlation_analysis import CorrelationAnalysis

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
    #MultiDimensionalBinning(features, 10)
    CorrelationAnalysis(features, list(body.morphological_measures.keys()))


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
    # PCAReduction(features)
    CorrelationAnalysis(features, list(body.morphological_measures.keys())).visualize()


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
    #MultiDimensionalBinning(features, 10)
    CorrelationAnalysis(features, list(body.morphological_measures.keys())).visualize()


def robogen_random_body():
    mutation = InsertMutation()
    features = []
    robots = []
    builder = RobogenBodyBuilder()

    for i in range(1000):
        body_builder = RandomRobogenBodyBuilder()
        body = body_builder.develop()
        feature_vector = body.morphological_measures.values()
        features.append(feature_vector)
    print(features)
    print(np.mean(features, axis=0))

    PCAReduction(features)
    MultiDimensionalBinning(features, 10)
    CorrelationAnalysis(features, list(body.morphological_measures.keys())).visualize()


def random():
    features = np.random.normal(0, 1, (10000, 8))

    #PCAReduction(features)
    #MultiDimensionalBinning(features, 5)
    CorrelationAnalysis(features, list(MorphologicalMeasures().keys())).visualize()


if __name__ == "__main__":
    robogen_random_body()
