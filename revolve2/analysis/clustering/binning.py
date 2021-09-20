import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy import stats
from scipy.cluster.vq import whiten


class MultiDimensionalBinning():

    def __init__(self, features, bins=5):
        whitened = whiten(np.array(features))
        H, edges = np.histogramdd(whitened, bins=bins)
        flat = H.flat
        flatten_H = flat[np.argwhere(flat)].flat
        sorted = np.sort(flatten_H)[::-1]
        print(sorted)
        plt.plot(sorted)
        plt.show()