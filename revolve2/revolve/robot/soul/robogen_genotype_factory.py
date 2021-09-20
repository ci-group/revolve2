import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

from revolve2.revolve.robot.body.robogen.random_robogen_body import RandomRobogenBodyBuilder, generate_graph


def random_tree(n: int = 15):
    pass


class RobogenGenotypeFactory:

    def __init__(self):
        body_builder = RandomRobogenBodyBuilder()
        random_body = body_builder.develop()
        self.graph = generate_graph(random_body.modules)


if __name__ == "__main__":
    factory = RobogenGenotypeFactory()
    a = np.random.randint(0, 2, size=(5, 5))
    print(a)
    D = nx.DiGraph(a)

    nx.draw_networkx(factory.graph)
    plt.draw()
    plt.show()
