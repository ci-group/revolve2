import numpy as np
from sklearn.neighbors import KNeighborsRegressor

from third_party.popi4sb.algorithms.evolutionary_model import EvolutionaryModel
from third_party.popi4sb.algorithms.k_nearest_neighbors import KNearestNeighbors


def estimate(x):
    mean = np.mean(x, 0, keepdims=True)
    z = np.expand_dims(x - mean, 2)

    S = np.mean(np.matmul(z, np.transpose(z, [0, 2, 1])), 0)
    covariance = np.linalg.cholesky(S)

    return mean, covariance


class EstimationDistributionAlgorithm(EvolutionaryModel):

    def __init__(self, config_method, config_model, calculate_fitness):
        super().__init__(config_method, config_model, calculate_fitness)
        print('Initialized EDA.')

    def _proposal(self, theta, E = None):
        mean, covariance = estimate(theta)
        return mean + np.dot(np.random.randn(theta.shape[0], theta.shape[1]), covariance)


class EstimationDistributionAlgorithmKNN(EstimationDistributionAlgorithm, KNearestNeighbors):

    def __init__(self,config_method, config_model, calculate_fitness , k_neighbors=3):
        super(EstimationDistributionAlgorithm).__init__(config_method, config_model, calculate_fitness)
        super(KNearestNeighbors).__init__(k_neighbors)
        print('Initialized EDA+knn.')

    def _proposal(self, theta, E = None):
        super(KNearestNeighbors).fit(theta, E)

        mean, covariance = estimate(theta)
        theta_new = mean + np.dot(np.random.randn(theta.shape[0] * 5, theta.shape[1]), covariance)

        return super(KNearestNeighbors).predict(theta, theta_new)
