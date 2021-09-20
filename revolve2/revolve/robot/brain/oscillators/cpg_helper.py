import numpy as np


# https://stackoverflow.com/questions/2572916/numpy-smart-symmetric-matrix
def symmetrize(a):
    """
    Return a symmetrized version of NumPy array a.

    Values 0 are replaced by the array value at the symmetric
    position (with respect to the diagonal), i.e. if a_ij = 0,
    then the returned array a' is such that a'_ij = a_ji.

    Diagonal values are left untouched.

    a -- square NumPy array, such that a_ij = 0 or a_ji = 0,
    for i != j.
    """
    return a + a.T - np.diag(a.diagonal())


def sigmoid_activation(x):
    return 1./(1.+np.exp(-x))


def tanh_activation(x):
    x_positive = np.exp(x)
    x_negative = np.exp(-x)

    return (x_positive - x_negative) / (x_positive + x_negative)


if __name__ == "__main__":
    print(symmetrize(np.random.random((3, 3))))
