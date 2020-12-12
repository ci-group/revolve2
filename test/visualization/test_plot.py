from matplotlib import pyplot as pl
import numpy as np


l = []
for _ in range(20):
    l.append(np.random.uniform(0, 1, 100))

mean = np.mean(l, axis=0)
standard_dev = np.std(l, axis=0)

pl.plot(mean)
pl.fill_between(range(100), mean-standard_dev, mean+standard_dev, alpha=0.5)
pl.show()