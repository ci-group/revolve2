from typing import List, Dict

from nca.core.abstract.structural.tree.tree_helper import Coordinate3D, Orientation
from nca.core.genome.genotype import Genotype
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body import RobotBody
from revolve.robot.body.body_builder import RobotBodyBuilder
from revolve.robot.body.robogen_body_measures import measure_robogen_body
from revolve.robot.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.robogen.robogen_module import RobogenModule


class RobogenBody(RobotBody):

    def __init__(self, axiom: RobogenModule):
        self.modules = [axiom]

    def add(self, module: RobogenModule):
        self.modules.append(module)

    def expandable_modules(self):
        number_of_orientations = len(Orientation.directions())
        available_modules = []

        for module in self.modules:
            if len(module.children.keys()) < number_of_orientations:
                available_modules.append(module)

        return available_modules


class RobogenBodyBuilder(RobotBodyBuilder):

    current_module: RobogenModule
    module_stack: List[RobogenModule]
    hashmap: Dict[Coordinate3D, RobogenModule]

    def __init__(self):
        super().__init__(RobogenBody)

    def _initialize(self):
        self.current_module: RobogenModule = RobogenModule()
        self.module_stack: List[RobogenModule] = []
        self.hashmap: Dict[Coordinate3D, RobogenModule] = {self.current_module.coordinate: self.current_module}

    def build(self, genotype: Genotype, ecosphere: Ecosphere = None):

        self._initialize()
        body = RobogenBody(self.current_module)

        for symbol in genotype.encoding:  # TODO

            if self._check_stacking(symbol):
                continue

            module = self._process_symbol(symbol)
            if module is not None:
                body.add(module)

        self.measures = measure_robogen_body(body, genotype.encoding)

        return body

    def _check_stacking(self, symbol):
        if symbol is RobogenSymbol.BRACKET_STASH:
            self.module_stack.append(self.current_module)
            return True

        if symbol is RobogenSymbol.BRACKET_POP:
            self.current_module = self.module_stack.pop()
            return True

        return False

    def _process_symbol(self, symbol):
        if symbol in RobogenSymbol.orientation():
            self.current_module.move_pointer(symbol.value)
            return None

        # Symbol in RobogenSymbol.modules():
        module = self.current_module.add_child(symbol)

        if module.coordinate not in self.hashmap.keys():
            self.current_module = module
            self.hashmap[module.coordinate] = self.current_module
            return module
        else:
            self.current_module = self.hashmap[module.coordinate]
            print("intersection limits ability to create module")
            return None
