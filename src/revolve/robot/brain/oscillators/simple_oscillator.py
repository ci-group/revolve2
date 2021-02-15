import math
import matplotlib.pyplot as plt

from abstract.configurations import RepresentationConfiguration
from abstract.sequential_identifier import SequentialIdentifier
from nca.core.genome.operators.initialization import GaussianInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation


class SimpleOscillator(ValuedRepresentation):

    identifier = SequentialIdentifier()

    def __init__(self):
        super().__init__(GaussianInitialization(), RepresentationConfiguration(4))
        self.id = self.identifier.id()

        self.time = 0.0
        self.x = 0.0

    def step(self, dt: float, x_input: float = 0.0):
        self.time += dt
        self.x = self[0] * math.sin(self[1] * self.time + self[2]) + self[3] + x_input

    def __eq__(self, other):
        return self.id == other.id

    def __repr__(self):
        return "oscillator Neuron: Weights (%f, %f, %f, %f) - Value (%f)" % (self[0], self[1], self[2], self[3], self.x)


if __name__ == "__main__":
    for i in range(1):
        neuron = SimpleOscillator()

        x_results = []

        for i in range(0, 300, 1):
            neuron.step(0.1)
            x_results.append(neuron.x)

        print(x_results)

        plt.plot(x_results, c="red")
        plt.show()
