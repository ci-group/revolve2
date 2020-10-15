from typing import List, Dict

from nca.core.abstract.structural.tree.tree_helper import Orientation, Coordinate3D
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body import RobotBody
from revolve.robot.robogen.rewritable_robogen_representation import RewritableRobogenRepresentation
from revolve.robot.robogen.robogen_grammar import RobogenModule, RobogenSymbol


class RobogenBody(RobotBody):

    core: RobogenModule = RobogenModule()

    def __init__(self):
        self.modules = []
        self.parents: List[RobogenModule] = []
        self.current: Dict = {}
        self.hashmap: Dict[Coordinate3D, RobogenModule] = {}

    def check_stacking(self, word):
        if word['module'] is RobogenSymbol.BRACKET_STASH:
            self.parents.append(self.current)
            return True

        if word['module'] is RobogenSymbol.BRACKET_POP:
            self.current = self.parents.pop()
            return True

        return False

    def develop(self, representation: RewritableRobogenRepresentation, ecosphere: Ecosphere = None):
        for word in representation.genome:

            if self.check_stacking(word):
                continue

            self.add_module(word)

        return self.modules

    def add_module(self, word):
        module_symbol = word['module']
        if module_symbol is RobogenSymbol.MODULE_CORE:
            coordinate = Coordinate3D(0, 0, 0)
        else:
            orientation = word['orientation'].value
            if orientation not in self.current['children']:
                return # print("Orientation not in children")
            coordinate = self.current['module'].coordinate + orientation
        module = RobogenModule(module_symbol, coordinate)

        # Availability children connectivity
        if module_symbol in [RobogenSymbol.MODULE_CORE, RobogenSymbol.MODULE_BLOCK]:
            children = dict()
            for child_orientation in Orientation:
                children[child_orientation] = coordinate + child_orientation
        elif module_symbol in [RobogenSymbol.MODULE_VERTICAL_JOINT, RobogenSymbol.MODULE_HORIZONTAL_JOINT]:
            # joints only allow to extend in the direct which they were originally created in.
            children = {orientation: coordinate + orientation}
        else:
            print("unable to create children")
            return

        # check for intersections
        if module.coordinate not in self.hashmap.keys():
            self.hashmap[module.coordinate] = module
            self.modules.append(module)
            self.current = {'module': module, 'children': children}
        else:
            print("intersection limits ability to create module")
