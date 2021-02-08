from math import sqrt
import numpy as np

import matplotlib.pyplot as plt

from nca.core.abstract.sequential_identifier import SequentialIdentifier
from revolve.robot.brain.oscillators.cpg_helper import tanh_activation


def RK4(f):
    return lambda t, y, dt: (
            lambda dy1: (
            lambda dy2: (
            lambda dy3: (
            lambda dy4: (dy1 + 2*dy2 + 2*dy3 + dy4)/6
            )( dt * f( t + dt  , y + dy3   ) )
	    )( dt * f( t + dt/2, y + dy2/2 ) )
	    )( dt * f( t + dt/2, y + dy1/2 ) )
	    )( dt * f( t       , y         ) )


class DifferentialCPG:

    identifier = SequentialIdentifier()

    def __init__(self):
        self.id = self.identifier.id()

        weight = np.random.normal()
        self.w_xy: float = weight
        self.w_yx: float = -weight
        self.w_o = 1

        bias = np.random.normal()
        self.x: float = bias
        self.y: float = -bias
        self.output: float = 0.0

        self.time: float = 0.0

        self.neural_function = np.sin
        self.activation_function = tanh_activation

    def step(self, dt: float, x_input: float = 0.0, y_input: float = 0.0):

        dx = self.w_yx * self.y * dt
        dy = self.w_xy * self.x * dt

        self.x = self.neural_function(self.x + dx)# + x_input
        self.y = self.neural_function(self.y + dy)# + y_input

        self.output = self.activation_function(self.x)

    def __eq__(self, other):
        return self.id == other.id

    def __repr__(self):
        return "CPG Neuron: Weights (%f, %f) - Value (%f, %f, %f)" % (self.w_xy, self.w_yx, self.x, self.y, self.output)


if __name__ == "__main__":
    for i in range(1):
        neuron = DifferentialCPG()

        x_results = []
        y_results = []
        z_results = []

        for i in range(0, 3000, 1):
            neuron.step(0.1)
            x_results.append(neuron.x)
            y_results.append(neuron.y)
            z_results.append(neuron.output)

        print(x_results)

        plt.plot(x_results, c="red")
        plt.plot(y_results, c="green")
        plt.plot(z_results, c="blue")
        plt.show()

