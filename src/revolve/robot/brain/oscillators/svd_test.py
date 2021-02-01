import numpy as np

mat = np.linalg.svd(np.array([[1, -0.5], [0.5, 1]]))
print(mat)