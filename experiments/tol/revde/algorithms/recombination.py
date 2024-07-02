import numpy as np
from ..utils.distributions import bernoulli


def recombination(x, bounds: tuple[int, int], F: float, CR: float):
    indices_1 = np.arange(x.shape[0])
    # take first parent
    x_1 = x[indices_1]
    # assign second parent (ensure)
    indices_2 = np.random.permutation(x.shape[0])
    x_2 = x_1[indices_2]
    # assign third parent
    indices_3 = np.random.permutation(x.shape[0])
    x_3 = x_2[indices_3]

    y_1 = np.clip(x_1 + F * (x_2 - x_3), bounds[0], bounds[1])
    y_2 = np.clip(x_2 + F * (x_3 - y_1), bounds[0], bounds[1])
    y_3 = np.clip(x_3 + F * (y_1 - y_2), bounds[0], bounds[1])

    # uniform crossover
    if CR < 1.:
        p_1 = bernoulli(CR, y_1.shape)
        p_2 = bernoulli(CR, y_2.shape)
        p_3 = bernoulli(CR, y_3.shape)
        y_1 = p_1 * y_1 + (1. - p_1) * x_1
        y_2 = p_2 * y_2 + (1. - p_2) * x_2
        y_3 = p_3 * y_3 + (1. - p_3) * x_3

    x[[indices_1, indices_2, indices_3]] = y_1, y_2, y_3
    return x
