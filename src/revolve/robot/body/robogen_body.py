from typing import List, Dict

from nca.core.abstract.structural.tree.tree_helper import Coordinate3D, Orientation
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body import RobotBody
from revolve.robot.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.robogen.robogen_module import RobogenModule


class RobogenBody(RobotBody):

    def __init__(self):
        self.current_module: RobogenModule = RobogenModule()
        self.modules: List[RobogenModule] = [self.current_module]
        self.module_stack: List[RobogenModule] = []
        self.hashmap: Dict[Coordinate3D, RobogenModule] = {self.current_module.coordinate: self.current_module}

    def check_stacking(self, symbol):
        if symbol is RobogenSymbol.BRACKET_STASH:
            self.module_stack.append(self.current_module)
            return True

        if symbol is RobogenSymbol.BRACKET_POP:
            self.current_module = self.module_stack.pop()
            return True

        return False

    def develop(self, encoding, ecosphere: Ecosphere = None):
        for symbol in encoding:

            if self.check_stacking(symbol):
                continue

            self.process(symbol)

        print("Expression efficiency: ", len(self.modules) / len(encoding))

        return self.modules

    def process(self, symbol):
        if symbol in RobogenSymbol.orientation():
            self.current_module.move_pointer(symbol.value)
            return True

        # Symbol in RobogenSymbol.modules():
        module = self.current_module.add_child(symbol)

        if module.coordinate not in self.hashmap.keys():
            self.current_module = module
            self.hashmap[module.coordinate] = self.current_module
            self.modules.append(module)
        else:
            self.current_module = self.hashmap[module.coordinate]
            print("intersection limits ability to create module")
            return False

        return True

    def expandable_modules(self):
        number_of_orientations = len(Orientation.directions())
        available_modules = []

        for module in self.modules:
            if len(module.children.keys()) < number_of_orientations:
                available_modules.append(module)

        return available_modules


    """
    def add_module(self, word):

        # setup module
        if word.module is not RobogenSymbol.MODULE_CORE:
            # Orientation correction for joints
            if self.current['orientation'] is Orientation.NEUTRAL:
                orientation = word.orientation.value
            else:
                orientation = self.current['orientation']

            # check if orientation is available for building
            if orientation not in self.current['children']:
                print("not in children")
                return False
            coordinate = self.current['children'][orientation]
        else:
            coordinate = Coordinate3D(0, 0, 0)

        if word.module in [RobogenSymbol.MODULE_CORE, RobogenSymbol.MODULE_BLOCK]:
            orientation = Orientation.NEUTRAL

        module = RobogenModule(word.module, coordinate)

        # Availability children connectivity
        children = dict()
        for child_orientation in Orientation.directions():
            children[child_orientation] = coordinate + child_orientation

        # Check for intersections
        if module.coordinate not in self.hashmap.keys():
            if len(self.current.keys()) > 0:
                self.current['module'].children.append(module)
            self.current = {'module': module, 'orientation': orientation, 'children': children}
            self.hashmap[module.coordinate] = self.current
            self.modules.append(module)
        else:
            self.current = self.hashmap[module.coordinate]
            print("intersection limits ability to create module")
            return False

        return True
    """
#orientation = Orientation.NEUTRAL # Blocks are not subjective to previous orientation constraints

#if module_symbol in [RobogenSymbol.MODULE_CORE, RobogenSymbol.MODULE_BLOCK]:
# elif module_symbol in [RobogenSymbol.MODULE_VERTICAL_JOINT, RobogenSymbol.MODULE_HORIZONTAL_JOINT]:
#    # joints only allow to extend in the direct which they were originally created in.
#    for child_orientation in Orientation.directions():
#        children[child_orientation] = coordinate + orientation
# else:
#    return False  # print("unable to create children")
# check for intersections
