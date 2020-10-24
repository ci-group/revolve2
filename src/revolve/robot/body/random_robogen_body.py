import numpy as np

from nca.core.abstract.structural.tree.tree_helper import Orientation
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.robogen_body import RobogenBody
from revolve.robot.robogen.robogen_grammar import RobogenSymbol, RobogenModule
from revolve.robot.robogen.robot_visualizer import generate_matrix, show


def _get_symbol_from_orientation(chosen_orientation):
    for orientation in RobogenSymbol.orientation():
        if orientation.value == chosen_orientation:
            return orientation
    return None


class RandomRobogenBody(RobogenBody):

    def develop(self):
        axiom_grammar = RobogenSymbol.generate_axiom()
        body = RobogenBody()
        body.develop(axiom_grammar)

        for i in range(10):
            available = body.expandable_modules()

            # pick element
            if len(available) == 0:
                break

            # pick element
            chosen_parent_module: RobogenModule = np.random.choice(available)
            chosen_orientation: Orientation = np.random.choice(chosen_parent_module.neighbor_orientations())
            orientation = _get_symbol_from_orientation(chosen_orientation)
            random_symbol: RobogenSymbol = np.random.choice(RobogenSymbol.modules())

            body.current_module = chosen_parent_module
            body.process(orientation)
            body.process(random_symbol)

        return body.modules


if __name__ == "__main__":
    body = RandomRobogenBody()
    modules = body.develop()

    body_matrix = generate_matrix(modules)
    show(body_matrix)

