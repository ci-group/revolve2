from __future__ import division, print_function
from pyrk import RK4
import numpy as np
import matplotlib.pyplot as plt


def vanderpol(t, xi, u):
    dx, x = xi
    mu = 4.0 # damping

    ddx = mu*(1-x**2)*dx-x
    dx = dx

    return np.array([ddx, dx])


rk = RK4(vanderpol)
t, y = rk.solve(np.array([-2, 1]), .01, 20)

y1 = []
y2 = []
for v in y:
    y1.append(v[0])
    y2.append(v[1])

plt.plot(y1, y2)
plt.ylabel('velocity')
plt.xlabel('position')
plt.grid(True)
plt.show()