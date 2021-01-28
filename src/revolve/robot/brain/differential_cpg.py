import numpy as np

from nca.core.abstract.sequential_identifier import SequentialIdentifier
from revolve.robot.brain.cpg_helper import tanh_activation


class DifferentialCPG:

    identifier = SequentialIdentifier()

    def __init__(self):
        self.id = self.identifier.id()

        weight = np.random.normal()
        self.x_weight: float = weight
        self.y_weight: float = -weight

        bias = np.random.normal()
        self.x_value: float = bias
        self.y_value: float = -bias
        self.activation_value: float = 0.0

        self.neural_function = np.sin
        self.activation_function = tanh_activation

    def step(self, dt: float, x_input: float = 0.0, y_input: float = 0.0):
        dx = self.y_weight * self.y_value * dt
        dy = self.x_weight * self.x_value * dt

        self.x_value = self.neural_function(self.x_value + dx + x_input)
        self.y_value = self.neural_function(self.y_value + dy + y_input)

        self.activation_value = self.activation_function(self.x_value)

    def __eq__(self, other):
        return self.id == other.id

    def __repr__(self):
        return "CPG Neuron: Weights (%f, %f) - Value (%f, %f, %f)" % (self.x_weight, self.y_weight, self.x_value, self.y_value, self.activation_value)


if __name__ == "__main__":
    for i in range(24):
        neuron = DifferentialCPG()

        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')

        x_results = []
        y_results = []
        z_results = []

        for i in range(0, 100, 1):
            x_results.append(neuron.x_value)
            y_results.append(neuron.y_value)
            z_results.append(neuron.activation_value)
            neuron.step(0.0)

        plt.plot(x_results, c="red")
        plt.plot(y_results, c="green")
        plt.plot(z_results, c="blue")
        plt.show()
