import math
from math import sqrt

import matplotlib.pyplot as plt

from revolve2.abstract.sequential_identifier import SequentialIdentifier
from revolve2.revolve.robot.brain.oscillators.cpg_helper import tanh_activation


class AukeCPG:

    identifier = SequentialIdentifier()

    def __init__(self):
        self.id = self.identifier.id()

        weight = 0.5  # np.random.normal()
        self.x_weight: float = weight
        self.y_weight: float = -weight
        self.output_weight = 1

        bias = -sqrt(2) / 2  # np.random.normal()
        self.x: float = bias
        self.y: float = -bias
        self.activation_value: float = 0.0

        #self.neural_function = np.sin
        self.activation_function = tanh_activation

    def step(self, x_input: float = 0.0, y_input: float = 0.0):
        r = math.sqrt(self.x**2 + self.y**2)
        mu = -1
        dx = (mu * self.x + self.y - self.x * r)
        dy = (-self.x - mu*self.y - self.y * r)
        print(r, dx, dy)

        self.x = self.x + dx + x_input
        self.y = self.y + dy + y_input

        self.activation_value = self.activation_function(self.x)

    def __eq__(self, other):
        return self.id == other.id

    def __repr__(self):
        return "CPG Neuron: Weights (%f, %f) - Value (%f, %f, %f)" % (self.x_weight, self.y_weight, self.x, self.y, self.activation_value)


if __name__ == "__main__":
    for i in range(1):
        neuron = AukeCPG()

        x_results = []
        y_results = []
        z_results = []

        for i in range(0, 100, 1):
            neuron.step(1)
            x_results.append(neuron.x)
            y_results.append(neuron.y)
            z_results.append(neuron.activation_value)

        print(x_results)

        plt.plot(x_results, c="red")
        plt.plot(y_results, c="green")
        plt.plot(z_results, c="blue")
        plt.show()

