from typing import Dict

from nca.core.abstract.sequential_identifier import NodeIdentifier
from nca.core.abstract.structural.tree.tree_helper import Orientation, Coordinate3D
from revolve.robot.robogen.robogen_grammar import RobogenSymbol


class RobogenModule:

    identifier = NodeIdentifier()
    symbol_type = RobogenSymbol

    def __init__(self, symbol: RobogenSymbol = RobogenSymbol.MODULE_CORE,
                 coordinate: Coordinate3D = Coordinate3D(0, 0, 0),
                 orientation: Orientation = Orientation.TOP,
                 parent_module=None,):
        self.id = self.identifier.id()
        self.symbol = symbol

        self.coordinate: Coordinate3D = coordinate
        self.pointer: Orientation = orientation

        self.children: Dict[Orientation, RobogenModule] = {}
        if parent_module is not None:
            self.children[Orientation.DOWN] = parent_module

    def add_child(self, symbol: RobogenSymbol):
        robogen_module = RobogenModule(symbol, self.coordinate + self.pointer, self.pointer)
        self.children[self.pointer] = robogen_module
        return robogen_module

    def move_pointer(self, pointer_orientation: Orientation):
        if self.symbol in RobogenSymbol.joints():
            return  # cannot change the orientation point for joints, only one is possible.

        self.pointer = pointer_orientation

    def __repr__(self):
        return "(" + self.symbol.name + ", " + str(self.id) + ", " + str(self.coordinate) + ", " + str(self.pointer) + ")"

    def neighbor_orientations(self):
        return list(set(Orientation.directions()) - set(self.children.keys()))

