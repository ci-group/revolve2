import numpy as np


# ----------------------------------------------------------------------------------------------------------------------
def bernoulli(p, shape):
    return np.random.binomial(1, p, shape)