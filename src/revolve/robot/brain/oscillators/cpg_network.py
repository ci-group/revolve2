from typing import List


from revolve.robot.brain.oscillators.cpg_helper import symmetrize
from revolve.robot.brain.oscillators.differential_cpg import DifferentialCPG

import numpy as np


class CPGNetwork:

    def __init__(self, adjacency_matrix: np.array = None, n: int = 3):
        self.n = n
        matrix_shape = (self.n, self.n)
        if adjacency_matrix is None:
            adjacency_matrix = np.ones(matrix_shape)

        self.adjacency_matrix = adjacency_matrix

        self.nodes: List[DifferentialCPG] = [DifferentialCPG() for _ in range(self.n)]
        self.weight_matrix = symmetrize(np.random.random(matrix_shape))

    def step(self, dt: float):
        input_values_x = np.zeros(self.weight_matrix.shape)
        input_values_y = np.zeros(self.weight_matrix.shape)

        for i, first_node in enumerate(self.nodes):
            for j, second_node in enumerate(self.nodes):
                if i == j:
                    continue

                if self.adjacency_matrix[i, j] == 0:
                    continue

                input_values_x[i, j] = first_node.x * self.weight_matrix[i, j] * dt

        for index, node in enumerate(self.nodes):
            inputs_x = sum(input_values_x[index, :])
            node.step(dt, inputs_x)


if __name__ == "__main__":
    for experiment in range(10):
        simple_network = CPGNetwork(n=5)

        import matplotlib.pyplot as plt

        fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')

        x_results = []
        y_results = []
        z_results = []

        for _ in range(2500):
            x_results.append(simple_network.nodes[0].output)
            y_results.append(simple_network.nodes[1].output)
            z_results.append(simple_network.nodes[2].output)
            simple_network.step(0.1)

        plt.plot(x_results, c="red")
        plt.plot(y_results, c="green")
        plt.plot(z_results, c="blue")
        plt.show()
