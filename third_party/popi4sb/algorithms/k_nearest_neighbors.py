from sklearn.neighbors import KNeighborsRegressor
import numpy as np


class KNearestNeighbors(object):

    def __init__(self, k_neighbors: int = 3):
        self.nn = KNeighborsRegressor(n_neighbors=k_neighbors)

        self.X = None
        self.E = None

    def fit(self, theta, E):
        if self.X is None:
            self.X = theta
            self.E = E
        elif self.X.shape[0] < 10000:
            self.X = np.concatenate((self.X, theta), 0)
            self.E = np.concatenate((self.E, E), 0)

        self.nn.fit(self.X, self.E)

    def predict(self, theta, theta_new):
        E_pred = self.nn.predict(theta_new)
        index = np.argsort(E_pred.squeeze())
        return theta_new[index[:theta.shape[0]]]