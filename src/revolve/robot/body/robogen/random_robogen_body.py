from typing import List

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

from nca.core.abstract.structural.tree.tree_helper import Orientation
from revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder, RobogenBody
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robogen_module import RobogenModule
from revolve.robot.body.robogen.robot_visualizer import generate_matrix, show


def _get_symbol_from_orientation(chosen_orientation):
    for orientation in RobogenSymbol.orientation():
        if orientation.value == chosen_orientation:
            return orientation
    return None


class RandomRobogenBodyBuilder(RobogenBodyBuilder):

    def __init__(self):
        super().__init__()

    def develop(self, size: int = 10):
        self._initialize()

        body = RobogenBody(self.current_module)

        for i in range(size):
            available = body.expandable_modules()

            # pick element
            if len(available) == 0:
                break

            # pick element
            chosen_parent_module: RobogenModule = np.random.choice(available)
            chosen_orientation: Orientation = np.random.choice(chosen_parent_module.neighbor_orientations())
            orientation = _get_symbol_from_orientation(chosen_orientation)
            random_symbol: RobogenSymbol = np.random.choice(RobogenSymbol.modules())

            self.current_module = chosen_parent_module

            O = self._process_symbol(orientation)
            module = self._process_symbol(random_symbol)
            print(module)
            if module is not None:
                print(module.id)
                body.add(module)

        print(body)

        return body


def generate_graph(modules: List[RobogenModule]):
    n = len(modules)
    adjacency_matrix = np.zeros((n, n))

    id_list = [module.id for module in modules]
    print(n, id_list)

    for module in modules:
        module_index = id_list.index(module.id)
        for child_key in module.children.keys():
            print(module.children[child_key].id)
            child_index = id_list.index(module.children[child_key].id)
            adjacency_matrix[module_index, child_index] = 1
            adjacency_matrix[child_index, module_index] = 1

    print(adjacency_matrix)
    D = nx.DiGraph(adjacency_matrix)
    return D


if __name__ == "__main__":
    np.random.seed(24)
    body_builder = RandomRobogenBodyBuilder()
    random_body = body_builder.develop()
    print(random_body.modules)


    body_matrix, connections, length, height = generate_matrix(random_body.modules)
    print(body_matrix)
    show(body_matrix, connections, length, height)

    generate_graph(random_body.modules)