import numpy as np
from sklearn.cluster import KMeans
from scipy.cluster.vq import whiten, vq, kmeans, kmeans2


class KMeans:

    def __init__(self, features, k):
        features = np.array(features)
        whitened_features = whiten(features)
        codebook, values = kmeans(whitened_features, k)
        cluster_assignment = vq(features, codebook)[0]

        for i in range(k):
            occurrences = np.count_nonzero(cluster_assignment == i)
            print(i, occurrences)

        features = np.array(features)
        whitened_features = whiten(features)
        kmeans = KMeans(n_clusters=4, init='k-means++', max_iter=300, n_init=10, random_state=0)
        pred_y = kmeans.fit_predict(whitened_features)
        plt.scatter(whitened_features[:, 0], whitened_features[:, 1])
        plt.scatter(kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1], s=300, c='red')
        plt.show()
        print(kmeans.cluster_centers_)