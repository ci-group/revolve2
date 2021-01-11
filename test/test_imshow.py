import matplotlib.pyplot as plt
import numpy as np


plt.figure()
im = plt.imshow(np.reshape(np.random.rand(100), newshape=(10, 10)),
                interpolation='none', vmin=0, vmax=1, aspect='equal')

ax = plt.gca()

# Major ticks
ax.set_xticks(np.arange(0, 10, 1))
ax.set_yticks(np.arange(0, 10, 1))

# Minor ticks
ax.set_xticks(np.arange(-.5, 10, 1), minor=True)
ax.set_yticks(np.arange(-.5, 10, 1), minor=True)

# Gridlines based on minor ticks
ax.grid(which='minor', color="white", linewidth=0.5)

plt.scatter([1.5, 2.5, 3.5, 4.5, 5.5], [1.0, 2.0, 3.0, 4.0, 5.0], color="black")

plt.show()