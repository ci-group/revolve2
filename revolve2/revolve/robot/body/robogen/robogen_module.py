from typing import Dict

from revolve2.abstract.sequential_identifier import NodeIdentifier
from revolve2.abstract.structural.tree.tree_helper import Orientation, Coordinate3D

from revolve2.revolve.robot.body.robogen.robogen_grammar import RobogenSymbol


class RobogenModule:

    identifier = NodeIdentifier()
    symbol_type = RobogenSymbol

    def __init__(self, symbol: RobogenSymbol = RobogenSymbol.MODULE_CORE,
                 coordinate: Coordinate3D = Coordinate3D(0, 0, 0),
                 orientation: Orientation = Orientation.TOP,
                 parent_module=None,):
        self.id: int = self.identifier.id()
        self.symbol: RobogenSymbol = symbol

        self.coordinate: Coordinate3D = coordinate
        self.orientation: Orientation = orientation
        self.next_orientation: Orientation = orientation

        self.children: Dict[Orientation, RobogenModule] = {}
        if parent_module is not None:
            self.children[Orientation.DOWN] = parent_module

    def next_coordinate(self):
        return self.coordinate + self.next_orientation.value

    def add_child(self, symbol: RobogenSymbol):
        robogen_module = RobogenModule(symbol, self.next_coordinate(), self.next_orientation)
        self.children[self.next_orientation] = robogen_module
        return robogen_module

    def move_pointer(self, pointer_orientation: Orientation):
        if self.symbol in RobogenSymbol.actuators():
            return  # cannot change the orientation point for joints, only one is possible.

        self.next_orientation = pointer_orientation

    def __repr__(self):
        return "(%s, %d, %s, %s)" % (self.symbol.name, self.id, str(self.coordinate), str(self.orientation))

    def neighbor_orientations(self):
        return list(set(Orientation.directions()) - set(self.children.keys()))
