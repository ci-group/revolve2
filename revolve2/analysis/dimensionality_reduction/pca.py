from sklearn.decomposition import PCA
import matplotlib.pyplot as plt


class PCAReduction:

    def __init__(self, data):
        pca = PCA(n_components=3)
        pca.fit(data)
        data_pca = pca.transform(data)
        data_new = pca.inverse_transform(data_pca)
        print(pca.explained_variance_ratio_)
        print(pca.singular_values_)

        # plot data
        fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')
        plt.scatter(data_pca[:, 0], data_pca[:, 1], alpha=1.0, s=10)
        plt.axis('auto')
        plt.show()