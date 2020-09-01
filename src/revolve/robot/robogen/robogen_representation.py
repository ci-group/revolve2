from typing import List, Dict
import numpy as np

from nca.core.abstract.structural.tree.tree_helper import Orientation, Coordinate3D
from nca.core.genome.representations.symbolic_representation import SymbolicRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenModule, RobogenSymbol


class RobogenRepresentation(SymbolicRepresentation):

    core: RobogenModule = RobogenModule()

    def __init__(self):
        super().__init__(symbols_type=RobogenModule.symbol_type)
        self.valid = True

    def _add(self, parent_module: RobogenModule, symbol: RobogenSymbol, orientation: Orientation):
        self.append(RobogenModule(symbol, parent_module.coordinate + orientation))

    def random_value(self):
        random_parent = np.random.choice(self)
        random_orientation = np.random.choice(Orientation)
        return RobogenModule(self.initialization(1)[0], random_parent.coordinate + random_orientation)

    def remove(self, module: RobogenModule):
        self.remove(module)

    def swap_indexes(self, indexes: List[int]):
        index1 = indexes[0]
        index2 = indexes[1]
        self[index1], self[index2] = self[index2], self[index1]

        tmp = self[index1].coordinate
        self[index1].coordinate = self[index2].coordinate
        self[index2].coordinate = tmp

    def is_valid(self):
        hashmap: Dict[Coordinate3D, RobogenModule] = {}
        self.valid = True

        for element in self:
            if element.coordinate not in hashmap.keys():
                hashmap[element.coordinate] = element
            else:
                self.valid = False
                break

        return self.valid
