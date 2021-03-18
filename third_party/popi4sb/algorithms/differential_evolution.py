import numpy as np

from third_party.popi4sb.algorithms.evolutionary_model import EvolutionaryModel
from third_party.popi4sb.algorithms.k_nearest_neighbors import KNearestNeighbors


class DE(EvolutionaryModel):
    def __init__(self, config_method, config_model, calculate_fitness):
        super().__init__(config_method, config_model, calculate_fitness)
        print('Initialized Differential Evolution (DE).')

    def _proposal(self, theta, E = None):
        indices_1 = np.random.permutation(theta.shape[0])
        indices_2 = np.random.permutation(theta.shape[0])
        theta_1 = theta[indices_1]
        theta_2 = theta[indices_2]

        de_noise = self.config_method['gamma'] * (theta_1 - theta_2)

        return super()._propose_theta(theta, de_noise, E)


class RevDE(DE):
    def __init__(self, config_method, config_model, calculate_fitness):
        super().__init__(config_method, config_model, calculate_fitness)
        print('Initialized RevDE.')

        gamma_squared = self.config_method['gamma'] ** 2
        gamma_cubed = self.config_method['gamma'] ** 3
        R = np.asarray([[1, self.config_method['gamma'], -self.config_method['gamma']],
                        [-self.config_method['gamma'], 1. - gamma_squared, self.config_method['gamma'] + gamma_squared],
                        [self.config_method['gamma'] + gamma_squared, -self.config_method['gamma'] + gamma_squared + gamma_cubed, 1. - 2. * gamma_squared - gamma_cubed]])

        self.R = np.expand_dims(R, axis=0)  # 1 x 3 x 3

    def _proposal(self, theta, E=None):
        theta_0 = np.expand_dims(theta, axis=1)  # B x 1 x D

        indices_1 = np.random.permutation(theta.shape[0])
        indices_2 = np.random.permutation(theta.shape[0])
        theta_1 = np.expand_dims(theta[indices_1], 1)
        theta_2 = np.expand_dims(theta[indices_2], 1)

        tht = np.concatenate((theta_0, theta_1, theta_2), axis=1) # B x 3 x D

        y = np.matmul(self.R, tht)

        theta_new = np.concatenate((y[:, 0], y[:, 1], y[:, 2]), axis=0)

        p_1 = np.random.binomial(1, self.config_method['CR'], theta_new.shape)
        return p_1 * theta_new + (1. - p_1) * np.concatenate((tht[:, 0], tht[:, 1], tht[:, 2]), axis=0)


class RevDEknn(RevDE, KNearestNeighbors):
    def __init__(self, config_method, config_model, calculate_fitness, neighbors=3):
        super(RevDE).__init__(config_method, config_model, calculate_fitness)
        super(KNearestNeighbors).__init__(neighbors)
        print('Initialized RevDE+knn.')

    def _proposal(self, theta, E = None):
        super(KNearestNeighbors).fit(theta, E)

        theta_new = super(RevDE)._proposal(theta, E)

        return super(KNearestNeighbors).predict(theta, theta_new)
