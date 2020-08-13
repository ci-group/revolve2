from typing import List, Dict

from nca.core.abstract.composite.tree_helper import Orientation, Coordinate3D
from nca.core.genome.representations.symbolic_representation import SymbolicRepresentation
from revolve.robot.body.robogen.robogen_grammar import RobogenModule, RobogenSymbol


class RobogenRepresentation(SymbolicRepresentation):

    core: RobogenModule = RobogenModule()

    def __init__(self):
        super().__init__(RobogenModule)
        self.valid = True

    def _initialize(self):
        self.genome: List[RobogenModule] = []

    def _add(self, parent_module: RobogenModule, symbol: RobogenSymbol, orientation: Orientation):
        self.genome.append(RobogenModule(symbol, parent_module.coordinate + orientation))

    def _remove(self, module: RobogenModule):
        self.genome.remove(module)

    def swap_indexes(self, indexes: List[int]):
        index1 = indexes[0]
        index2 = indexes[1]
        self.genome[index1], self.genome[index2] = self.genome[index2], self.genome[index1]

        tmp = self.genome[index1].coordinate
        self.genome[index1].coordinate = self.genome[index2].coordinate
        self.genome[index2].coordinate = tmp

    def is_valid(self):
        hashmap: Dict[Coordinate3D, RobogenModule] = {}
        self.valid = True

        for element in self.genome:
            if element.coordinate not in hashmap.keys():
                hashmap[element.coordinate] = element
            else:
                self.valid = False
                break

        return self.valid