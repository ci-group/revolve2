# importing libraries
from typing import List

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


# Loading the dataset
class CorrelationAnalysis:

    def __init__(self, features: np.array, feature_names: List[str]):
        df = pd.DataFrame(features, columns=feature_names)
        print(df.head())

        self.cor = df.corr()

    def visualize(self):
        sns.heatmap(self.cor, annot=True, cmap=plt.cm.turbo)

        plt.show()
